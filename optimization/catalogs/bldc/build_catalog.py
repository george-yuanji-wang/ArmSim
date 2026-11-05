"""Normalize researched manufacturer facts; never impute missing ratings."""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent
MANIFEST = {row['source_id']: row for row in json.loads((ROOT/'sources/manifest.json').read_text())}
COLUMNS = ('manufacturer','model','motor_id','construction','voltage_V','winding',
    'rated_torque_Nm','rated_speed_rpm','rated_power_W','peak_torque_Nm','peak_duration_s',
    'stall_torque_Nm','no_load_speed_rpm','max_speed_rpm','mass_kg','outer_diameter_mm',
    'body_length_mm','frame_width_mm','frame_height_mm','cylindrical_envelope_diameter_mm',
    'envelope_basis','overall_axial_length_mm','rotor_inertia_kg_m2','rated_current_A',
    'peak_current_A','torque_constant_Nm_A','phase_resistance_ohm','phase_inductance_mH',
    'nominal_size_mm','temperature_conditions','availability','source_url',
    'secondary_source_url','source_locator','retrieved_date','notes')

def base(manufacturer, model, source_id, **values):
    row = dict.fromkeys(COLUMNS, '')
    row.update(manufacturer=manufacturer,model=model,motor_id=manufacturer.upper()+'_'+model.replace(' ','_').replace('/','_'),
        source_url=MANIFEST[source_id]['url'],source_locator='Official product electrical specification and dimension table',
        retrieved_date='2026-11-01',availability='Listed by manufacturer; stock/lead time not confirmed')
    row.update(values)
    return row

rows=[]
cube_common=dict(construction='frameless inrunner stator/rotor set',
    temperature_conditions='Ambient -20 to 50 C; continuous-rating heatsink/mounting test conditions not specified',
    availability='Official manufacturer product listing; stock/lead time not confirmed')

rows.append(base('CubeMars','RI30 KV260 24V','cubemars_ri30',**cube_common,
    voltage_V=24,winding='KV260; winding connection not specified',rated_torque_Nm=.076,
    rated_speed_rpm=4110,rated_power_W=33,peak_torque_Nm=.23,no_load_speed_rpm=6240,
    mass_kg=.040,outer_diameter_mm=38,body_length_mm=14,cylindrical_envelope_diameter_mm=38,
    envelope_basis='Published stator/rotor envelope; housing, bearings and encoder excluded',
    rotor_inertia_kg_m2=4.23e-7,rated_current_A=1.98,peak_current_A=7.72,
    torque_constant_Nm_A=.03673,phase_resistance_ohm=2.2,phase_inductance_mH=1.410,
    nominal_size_mm=30,notes='Page labels 24 V as Rated Power V; interpreted as voltage from the unit/context. Peak duration, stall torque and mechanical speed limit are unpublished.'))

for family,source_id,od,body,mass,tau,peak,inertia,current,pc,kt,res,ind,kv,winding,speeds,noload in (
    ('RI50','cubemars_ri50',54,27,.1808,.58,1.67,22.8e-7,4.8,14.8,.120,1.420,1.500,100,'star',(1090,1860,2600),(2004,3006,4008)),
    ('RI60','cubemars_ri60',60,23,.1559,.57,1.63,33.05e-7,5.6,16.8,.100,.900,.8775,120,'delta',(1440,2320,3190),(2532,3798,5064)),
):
    for voltage,speed,nl in zip((24,36,48),speeds,noload):
        rows.append(base('CubeMars',f'{family} KV{kv} {voltage}V',source_id,**cube_common,
            voltage_V=voltage,winding=f'KV{kv}; {winding}',rated_torque_Nm=tau,rated_speed_rpm=speed,
            peak_torque_Nm=peak,no_load_speed_rpm=nl,mass_kg=mass,outer_diameter_mm=od,
            body_length_mm=body,cylindrical_envelope_diameter_mm=od,
            envelope_basis='Published stator/rotor envelope; housing, bearings and encoder excluded',
            rotor_inertia_kg_m2=inertia,rated_current_A=current,peak_current_A=pc,
            torque_constant_Nm_A=kt,phase_resistance_ohm=res,phase_inductance_mH=ind,
            nominal_size_mm=int(family[2:]),
            notes='Voltage-specific operating point of the same winding, not a distinct physical motor. Family comparison power (66 W RI50 / 85 W RI60) has no explicit voltage basis and is therefore not assigned to these rows. Peak duration, stall torque and mechanical speed limit unpublished.'))

for source_id,label,body in (('cubemars_ri75_welded','Welded',34),('cubemars_ri75_connector','With Connector',36.2)):
    rows.append(base('CubeMars',f'RI75-PH KV70 {label} 48V',source_id,
        construction='potted frameless inrunner stator/rotor set',voltage_V=48,winding='KV70; delta',
        rated_torque_Nm=1.2,rated_speed_rpm=2365,rated_power_W=297,peak_torque_Nm=3.8,
        no_load_speed_rpm=3360,mass_kg=.348,outer_diameter_mm=70,body_length_mm=body,
        cylindrical_envelope_diameter_mm=70,envelope_basis='Published potted stator/rotor envelope; custom bearings/housing excluded',
        rotor_inertia_kg_m2=416e-7,rated_current_A=7.28,peak_current_A=24.2,
        torque_constant_Nm_A=.143,nominal_size_mm=75,
        temperature_conditions='Ambient -20 to 85 C; continuous-rating heatsink/mounting test conditions not specified',
        notes='Named RI75 but actual published OD is 70 mm. 3360 rpm at 48 V is from Core Data/comparison; detailed table also gives 1602 rpm at 24 V. Optional R/L left blank: detailed table 453 mOhm/485 uH conflicts with comparison 495 mOhm/673 uH. Peak current is labeled Ref.'))

for name,source_id,od,body,mass,tau,peak,power,speed,nl,inertia,current,pc,kt,res,ind,kv in (
    ('RO40 Lite','cubemars_ro40_lite',46.3,13,.060,.2,.6,51,2450,3400,122.11e-7,2.85,11.6,.068,1.700,.580,140),
    ('RO50 Lite','cubemars_ro50_lite',61.7,14.5,.105,.3,.9,66,2100,2600,327.21e-7,3.4,9.3,.088,1.035,.460,108),
):
    rows.append(base('CubeMars',f'{name} KV{kv} 24V',source_id,
        construction='frameless outrunner stator/rotor set without Hall sensors',
        voltage_V=24,winding=f'KV{kv}; connection not specified',rated_torque_Nm=tau,
        rated_speed_rpm=speed,rated_power_W=power,peak_torque_Nm=peak,no_load_speed_rpm=nl,
        mass_kg=mass,outer_diameter_mm=od,body_length_mm=body,cylindrical_envelope_diameter_mm=od,
        envelope_basis='Published stator/rotor envelope; outer casing/sensors/bearings excluded',
        rotor_inertia_kg_m2=inertia,rated_current_A=current,peak_current_A=pc,
        torque_constant_Nm_A=kt,phase_resistance_ohm=res,phase_inductance_mH=ind,
        nominal_size_mm=int(name[2:4]),temperature_conditions=cube_common['temperature_conditions'],
        notes='Source Kt/Kv, phase/DC-current conventions retained as published. Peak duration, stall torque and mechanical speed limit unpublished.'))

for model,voltage,tau,speed,power,peak,nl,mass,body,inertia,current,pc,kt,res,ind in (
    ('DB42S01',48,.05,6000,31,.15,7700,.25,41,2.4e-6,.88,2.63,.057,3.5,5.8),
    ('DB42S02',17,.05,8000,42,.15,12000,.25,41,2.4e-6,3.57,10.78,.014,.19,.245),
    ('DB42S03',24,.0625,4000,26,.19,6200,.30,41,2.4e-6,1.79,5.4,.035,1.5,2.1),
    ('DB42M01',48,.11,6000,69,.33,7800,.45,60.3,4.8e-6,2.12,5.77,.052,1.3,2.6),
    ('DB42M03',24,.125,4000,52,.38,6000,.45,60.3,4.8e-6,3.47,10.6,.036,.8,1.2),
    ('DB42L01',24,.18,4000,75,.56,6100,.65,80.3,6.68e-6,5.14,15.5,.036,.43,.7),
):
    rows.append(base('Nanotec',model,'nanotec_'+model.lower(),construction='framed internal-rotor BLDC with Hall sensors',
        voltage_V=voltage,winding='delta',rated_torque_Nm=tau,rated_speed_rpm=speed,rated_power_W=power,
        peak_torque_Nm=peak,no_load_speed_rpm=nl,mass_kg=mass,body_length_mm=body,
        frame_width_mm=42,frame_height_mm=42,cylindrical_envelope_diameter_mm=math.hypot(42,42),
        envelope_basis='Conservative derived bounding circle for published 42 x 42 mm square flange; nominal dimensions',
        overall_axial_length_mm=body+24+13.5,rotor_inertia_kg_m2=inertia,rated_current_A=current,
        peak_current_A=pc,torque_constant_Nm_A=kt,phase_resistance_ohm=res,phase_inductance_mH=ind,
        nominal_size_mm=42,temperature_conditions='Ambient -10 to 50 C; maximum temperature rise 80 C; mounting condition not specified',
        secondary_source_url=MANIFEST['nanotec_db42_overview']['url'],source_locator='Individual drawing PDF p1, specification table and three-view dimensions; datasheet preferred over rounded family overview',
        notes='Body length excludes front/rear shafts; overall axial length derived by adding published 24 mm front and 13.5 mm rear extensions. 42 mm is flange width, not motor OD. Peak torque is not stall torque; duration/mechanical maximum speed unpublished.'+(' Body passes <100 mm filter but shaft-to-shaft length exceeds 100 mm.' if body+37.5>=100 else '')))

for model,voltage,tau,power,mass,body,inertia,current,pc,kt in (
    ('DB43M024030-A',24,.17,53,.6,60.3,60e-7,3.1,9.3,.055),
    ('DB43M048030-A',48,.17,53,.6,60.3,60e-7,1.5,4.6,.11),
    ('DB43L024030-A',24,.30,94,.8,80.3,80e-7,4.8,14.4,.0625),
    ('DB43L048030-A',48,.30,94,.8,80.3,80e-7,2.4,7.2,.125),
):
    rows.append(base('Nanotec',model,'nanotec_db43_overview',construction='framed internal-rotor BLDC; base Hall-sensor version',
        voltage_V=voltage,winding='connection not verified for -A variant',rated_torque_Nm=tau,
        rated_speed_rpm=3000,rated_power_W=power,mass_kg=mass,body_length_mm=body,
        frame_width_mm=42,frame_height_mm=42,cylindrical_envelope_diameter_mm=math.hypot(42,42),
        envelope_basis='Conservative derived bounding circle for published 42 x 42 mm square flange; nominal dimensions',
        rotor_inertia_kg_m2=inertia,rated_current_A=current,peak_current_A=pc,
        torque_constant_Nm_A=kt,nominal_size_mm=42,
        secondary_source_url='https://www.nanotec.com/eu/de/produkte/2003-db43-buerstenloser-dc-motor',
        source_locator='Official family overview PDF p1 versions table; live DB43 family list maps base model to -A SKU',
        notes='Overview omits -A suffix while live manufacturer SKU includes -A; listed base dimensions/rating matched. Body length excludes shaft projection. Individual -A PDF was not accessible; no peak torque inferred from peak current.'))

rows.append(base('Nanotec','DBL42L024032-A','nanotec_dbl42_overview',
    construction='framed cylindrical internal-rotor BLDC with Hall sensors',voltage_V=24,
    rated_torque_Nm=.12,rated_speed_rpm=3200,rated_power_W=40,mass_kg=.35,
    outer_diameter_mm=42.2,body_length_mm=60,cylindrical_envelope_diameter_mm=42.2,
    envelope_basis='Published maximum cylindrical body diameter; body length A excludes shaft projections',
    rotor_inertia_kg_m2=44e-7,rated_current_A=2.1,peak_current_A=6.3,
    torque_constant_Nm_A=.054,nominal_size_mm=42,
    source_locator='Official family overview PDF p1 versions table; dimension drawing states OD42.2 max',
    notes='Published peak current is not converted into peak/stall torque. Peak duration, stall torque, no-load and mechanical speed limits unpublished in this source.'))

excluded = [
    dict(model='CubeMars RI20 KV600',outer_diameter_mm=25,body_length_mm=14.5,reason='Below 30 mm OD',source_url=MANIFEST['cubemars_ri30']['url']),
    dict(model='CubeMars RI70 KV95',outer_diameter_mm=76,body_length_mm=24,reason='Nominal name 70; actual OD exceeds 70 mm',source_url=MANIFEST['cubemars_ri50']['url']),
    dict(model='CubeMars RO60 Standard',outer_diameter_mm=73.8,body_length_mm=23,reason='Actual OD exceeds 70 mm',source_url='https://www.cubemars.com/categorys/frameless-outrunner-torque-motor'),
    *[dict(model='Nanotec '+m,outer_diameter_mm='',body_length_mm=100,reason='Body length is 100 mm, not strictly less than 100 mm',source_url=MANIFEST['nanotec_db42_overview']['url']) for m in ('DB42C01','DB42C02','DB42C03')],
    dict(model='Nanotec DB43C048030-A',outer_diameter_mm='',body_length_mm=100.3,reason='Body length exceeds 100 mm',source_url=MANIFEST['nanotec_db43_overview']['url']),
]

def write_csv(path,columns,data):
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader();writer.writerows(data)

if __name__=='__main__':
    assert len(rows)==22 and len({r['motor_id'] for r in rows})==22
    for row in rows:
        assert 30 <= row['cylindrical_envelope_diameter_mm'] <= 70
        assert row['body_length_mm']<100 and row['rated_torque_Nm']>0
        assert row['max_speed_rpm']=='' and row['stall_torque_Nm']=='' and row['peak_duration_s']==''
    write_csv(ROOT/'motors.csv',COLUMNS,rows)
    write_csv(ROOT/'excluded_motors.csv',('model','outer_diameter_mm','body_length_mm','reason','source_url'),excluded)
    (ROOT/'motors.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(f'{len(rows)} voltage-specific records, 18 distinct physical model/connector variants, {len(excluded)} explicit exclusions.')
