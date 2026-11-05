"""Combine sourced motor surveys without inventing unavailable specifications.

Run from the repository root with ``python -m optimization.catalogs.build_catalogs``.
Research catalogs remain separate from active optimizer inputs.
"""
from __future__ import annotations

from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re

from optimization.inputs import MOTOR_COLUMNS, load_motors

ROOT = Path(__file__).resolve().parent
INPUTS = (ROOT/'bldc/motors.csv', ROOT/'maxon/motors.csv',
          ROOT/'stepperonline/motors.json')
NUMERIC_FIELDS = {
    'voltage_V','rated_torque_Nm','rated_speed_rpm','rated_power_W','peak_torque_Nm',
    'peak_duration_s','stall_torque_Nm','no_load_speed_rpm','max_speed_rpm','mass_kg',
    'outer_diameter_mm','body_length_mm','frame_width_mm','frame_height_mm',
    'cylindrical_envelope_diameter_mm','overall_axial_length_mm','rotor_inertia_kg_m2',
    'rated_current_A','peak_current_A','torque_constant_Nm_A','phase_resistance_ohm',
    'phase_inductance_mH','nominal_size_mm',
}
EXTRA_FIELDS = ('research_source_catalog','size_filter_pass','complete_axial_envelope_status',
                'optimizer_ready','optimizer_missing_fields')


def _stepperonline_rows(document):
    """Map supplier facts into the common schema, retaining every original field.

    A body-plus-shafts lower bound is not a verified complete axial envelope.
    Missing rotor inertia remains unknown here; a study adapter may separately
    select an explicitly labeled evaluation bound.
    """
    rows = []
    for source in document['motors']:
        square = source.get('body_shape') == 'square'
        radius = source.get('derived_circumscribed_radius_mm')
        row = {
            **source,
            'construction': source['assembly_type'],
            'voltage_V': source['rated_voltage_V'],
            'outer_diameter_mm': source.get('body_diameter_mm'),
            'frame_width_mm': source.get('body_width_mm') if square else None,
            'frame_height_mm': source.get('body_height_mm') if square else None,
            'cylindrical_envelope_diameter_mm': None if radius is None else 2*radius,
            'envelope_basis': source['derived_circumscribed_radius_definition'],
            'overall_axial_length_mm': None,
            'source_locator': 'stepperonline/'+source['source_snapshot'],
        }
        for key in sorted(NUMERIC_FIELDS):
            value = row.get(key)
            row[key] = float(value) if value not in ('', None) else None
            if row[key] is not None and not math.isfinite(row[key]):
                raise ValueError(f"Nonfinite value: {row['motor_id']} {key}")
        rows.append(row)
    return rows


def _read_sources():
    records, fields, manifest = [], [], []
    for path in INPUTS:
        content=path.read_bytes()
        if path.suffix == '.json':
            document = json.loads(content)
            for row in _stepperonline_rows(document):
                for key in row:
                    if key not in fields and key not in EXTRA_FIELDS:
                        fields.append(key)
                row['research_source_catalog'] = path.relative_to(ROOT).as_posix()
                records.append(row)
            manifest.append({'path':path.relative_to(ROOT).as_posix(),
                             'sha256':hashlib.sha256(content).hexdigest()})
            continue
        with path.open(newline='',encoding='utf-8-sig') as stream:
            reader=csv.DictReader(stream)
            for key in reader.fieldnames or ():
                key='overall_axial_length_mm' if key=='total_axial_envelope_mm' else key
                if key not in fields: fields.append(key)
            for original in reader:
                row=dict(original)
                if 'total_axial_envelope_mm' in row:
                    old=row.pop('total_axial_envelope_mm')
                    current=row.get('overall_axial_length_mm','')
                    if current and old and float(current)!=float(old):
                        raise ValueError(f"Conflicting axial length for {row['motor_id']}")
                    row['overall_axial_length_mm']=current or old
                for key in NUMERIC_FIELDS:
                    value=row.get(key,'')
                    row[key]=float(value) if value not in ('',None) else None
                    if row[key] is not None and not math.isfinite(row[key]):
                        raise ValueError(f"Nonfinite value: {row['motor_id']} {key}")
                row['research_source_catalog']=path.relative_to(ROOT).as_posix()
                records.append(row)
        manifest.append({'path':path.relative_to(ROOT).as_posix(),
                         'sha256':hashlib.sha256(content).hexdigest()})
    return records,fields,manifest


def _optimizer_row(row):
    """Return strict Motor-schema values plus a list of missing prerequisites."""
    mapped={key:row.get(key) for key in MOTOR_COLUMNS}
    mapped['name']=f"{row['manufacturer']} {row['model']} ({row['voltage_V']:g} V)"
    diameter=row.get('cylindrical_envelope_diameter_mm')
    mapped['radius_mm']=None if diameter is None else diameter/2
    mapped['length_mm']=row.get('body_length_mm')
    optional={'rated_power_W','peak_torque_Nm','peak_duration_s'}
    missing=[key for key in MOTOR_COLUMNS if key not in optional and mapped.get(key) in ('',None)]
    peak=mapped.get('peak_torque_Nm')
    if peak is not None and peak>mapped['rated_torque_Nm'] and mapped.get('peak_duration_s') is None:
        missing.append('peak_duration_s')
    return mapped,missing


def _write_csv(path,fields,rows):
    def cell(value):
        return (json.dumps(value, ensure_ascii=False, allow_nan=False)
                if isinstance(value, (list, dict)) else value)
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        writer.writerows({key:cell(row.get(key)) for key in fields} for row in rows)


def build_catalogs(output_dir=ROOT):
    """Rebuild combined research tables without altering supplier facts or inputs."""
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    records,fields,manifest=_read_sources()
    if len({row['motor_id'] for row in records})!=len(records):
        raise ValueError('Duplicate motor IDs in research catalogs')
    ready,audits=[],[]
    for row in records:
        envelope=row['cylindrical_envelope_diameter_mm']
        length=row['body_length_mm']
        size_pass=envelope is not None and 30<=envelope<=70 and length is not None and 0<length<100
        row['size_filter_pass']=bool(size_pass)
        overall=row.get('overall_axial_length_mm')
        lower_bound = row.get('derived_named_axial_span_lower_bound_mm')
        row['complete_axial_envelope_status']=(
             ('lower bound >=100 mm; full envelope not established' if lower_bound >= 100 else
              'lower bound only; full envelope not established')
             if overall is None and lower_bound is not None else
             'not established' if overall is None else
             'under 100 mm including shafts' if overall<100 else '100 mm or more including shafts')
        mapped,missing=_optimizer_row(row)
        row['optimizer_ready']=not missing and bool(size_pass)
        row['optimizer_missing_fields']='; '.join(missing)
        if row['optimizer_ready']: ready.append(mapped)
        audits.append({'motor_id':row['motor_id'],'source_url':row['source_url'],
            'optimizer_ready':row['optimizer_ready'],'missing_required_fields':missing,
            'size_filter_pass':bool(size_pass),'complete_axial_envelope_status':row['complete_axial_envelope_status']})
    records.sort(key=lambda row:(row['manufacturer'].casefold(),row['model'].casefold(),row['voltage_V']))
    _write_csv(output_dir/'motors.csv',fields+list(EXTRA_FIELDS),records)
    _write_csv(output_dir/'motors_optimizer_ready.csv',MOTOR_COLUMNS,ready)
    # The application's own parser independently verifies the ready subset.
    parsed=load_motors(output_dir/'motors_optimizer_ready.csv')
    if len(parsed)!=len(ready): raise ValueError('Optimizer-ready row count changed during validation')
    physical_ids={(row['manufacturer'].casefold(),
                   re.sub(r'\s+\d+V$','',row['model']) if row['manufacturer']=='CubeMars' else row['model'])
                  for row in records}
    audit={
        'retrieved_date':max(row['retrieved_date'] for row in records),
        'retrieval_dates':sorted({row['retrieved_date'] for row in records}),
        'motor_records':len(records),
        'physical_model_or_winding_variants':len(physical_ids),
        'manufacturer_record_counts':dict(Counter(row['manufacturer'] for row in records)),
        'optimizer_ready_records':len(ready),'optimizer_incomplete_records':len(records)-len(ready),
        'body_size_filter':{'minimum_cylindrical_envelope_diameter_mm':30,
                            'maximum_cylindrical_envelope_diameter_mm':70,
                            'maximum_body_length_mm_exclusive':100},
        'body_size_filter_counts':dict(Counter('pass' if row['size_filter_pass'] else 'outside'
                                              for row in records)),
        'complete_axial_envelope_counts':dict(Counter(row['complete_axial_envelope_status'] for row in records)),
        'missing_field_counts':dict(Counter(key for row in audits for key in row['missing_required_fields'])),
        'input_catalogs':manifest,'motors':audits,
        'simulation_status':'Research catalog rebuild does not replace optimizer inputs or run simulations.',
        'reducer_catalog':'sitodrive/reducer_variants.csv',
        'reducer_readiness':'Public reducer specifications must be checked for every required optimizer field; output torque divided by ratio is not a published input-torque limit.',
    }
    reducer_coverage=ROOT/'sitodrive/coverage.json'
    if reducer_coverage.exists():
        audit['sitodrive_coverage']=json.loads(reducer_coverage.read_text(encoding='utf-8'))
    (output_dir/'motors.json').write_text(json.dumps(records,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    (output_dir/'catalog_audit.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({key:audit[key] for key in ('motor_records','physical_model_or_winding_variants','optimizer_ready_records','optimizer_incomplete_records','manufacturer_record_counts')},indent=2))
    return audit


if __name__=='__main__':
    build_catalogs()
