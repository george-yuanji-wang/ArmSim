# Lightweight motor candidates

Checked November 5, 2026.

[Open the filterable Excel catalog](outputs/20261105/motor_candidates.xlsx). The Motors sheet contains the comparison table; Conditions retains measurement definitions, source disagreements and encoder options.

The survey contains **45 model, winding or voltage records** across the manufacturers below. It covers their inspected public families and selected prototype alternatives. Encoder options are retained within the supplier JSON rather than counted as independent motors.

The arm's current targets require operation through **1,500 motor rpm**: J1–J3 at 50:1 and 180 degrees/s, J4–J6 at 30:1 and 300 degrees/s. A published loaded point establishes a relevant operating point, not an optimized or validated arm design.

Nominal, rated, quoted load, stall and peak torque retain their separate meanings. Unloaded speed is never paired with a separately measured torque as if it were one rated point. Blank numeric fields mean unpublished. Prices remain in their original currencies; no exchange rate or thermal rating is invented.

## Most useful candidates

- **CubeMars GL40 KV210:** strongest documented fast winding in this survey; 0.25 Nm at 2,100 rpm, 107 g. J1/J3/wrist candidate and a conditional J2 option after mass/duty reassessment. Above the preferred USD50 budget.
- **CubeMars GL30 KV290:** 0.08 Nm at 1,990 rpm, 41 g; light wrist/J6 candidate whose peak duty and actual arm loads still need assessment.
- **iPower GM3506:** inexpensive 64 g research candidate. Its quoted torque test is at 12 V and unloaded speed at 16 V; loaded 1,500 rpm torque and allowed supply need confirmation.
- **CubeMars GL60 KV55:** higher-torque J2 candidate with a slower documented operating point. Higher-voltage capability is a manufacturer inquiry, not an assumed rating.
- **SMC PM4310/4315/5215 and Reacher GB4310/5210:** useful lightweight motor packages for higher-speed winding inquiries. Their stock published windings are slower than the current target.

## Evidence categories

- **Loaded rating above 1500 rpm: 2 records.**
- **1500 rpm curve point only: 1 records.**
- **Confirm voltage or winding: 25 records.**
- **Missing loaded-speed data: 11 records.**
- **Outside preferred envelope: 6 records.**

## Loaded rating above 1500 rpm

| ID | Manufacturer / motor | Torque (Nm; meaning shown) | Speed evidence (rpm) | Mass (g) | Listed price |
|---|---|---|---|---:|---:|
| LM001 | [CubeMars GL30 (KV290 marketed; table Kv255)](https://www.cubemars.com/product/gl30-kv290-gimbal-motor.html) | 0.08 rated | 1990 loaded at 12 V | 41 | 50.99 USD |
| LM002 | [CubeMars GL40 (KV210)](https://www.cubemars.com/product/gl40-kv210-gimbal-motor.html) | 0.25 rated | 2100 loaded at 16 V | 107 | 72.99 USD |

## 1500 rpm curve point only

| ID | Manufacturer / motor | Torque (Nm; meaning shown) | Speed evidence (rpm) | Mass (g) | Listed price |
|---|---|---|---|---:|---:|
| LM003 | [CubeMars GL40 II (KV82.5)](https://www.cubemars.com/product/gl40-II-kv82-5-gimbal-motor.html) | ~0.12 curve point | ~1500 curve at 24 V | 125 | 134 USD |

## Confirm voltage or winding

| ID | Manufacturer / motor | Torque (Nm; meaning shown) | Speed evidence (rpm) | Mass (g) | Listed price |
|---|---|---|---|---:|---:|
| LM004 | [CubeMars GL35 (KV100 marketed; table Kv82.5)](https://www.cubemars.com/product/gl35-kv100-gimbal-motor.html) | 0.15 rated | 815 loaded at 16 V | 90 | 54.99 USD |
| LM005 | [CubeMars GL40 (KV70)](https://www.cubemars.com/product/gl40-kv70-gimbal-motor.html) | 0.25 rated | 430 loaded at 16 V | 107 | 72.99 USD |
| LM006 | [CubeMars GL60 (KV25)](https://www.cubemars.com/goods.php?id=1133) | 0.6 rated | 310 loaded at 24 V | 230 | 109 USD |
| LM007 | [CubeMars GL60 (KV55)](https://www.cubemars.com/images/file/20230703/1688352405877552.pdf) | 0.6 rated | 840 loaded at 24 V | 226 | Quote / unpublished |
| LM008 | [Huizhou Techno Motor Co., Ltd. (TechnoMotor / TejiaMotor product sites) GBM3506H-130T](https://www.tejiamotor.com/Pr_d_gci_112_id_123.html) | 0.13 stall | Loaded speed unpublished | 61.6 | Quote / unpublished |
| LM009 | [Huizhou Techno Motor Co., Ltd. (TechnoMotor / TejiaMotor product sites) GBM5208H-200T](https://www.technomotor.com/En_Pr_d_gci_112_id_128.html) | 0.368 stall | Loaded speed unpublished | 196.7 | Quote / unpublished |
| LM010 | [Reacher Technology GB3510 (40T; published speed constant45rpm/V)](https://www.reacherbrushless.com/product/product-73-87.html) | 0.11 nominal | 363 loaded at 12 V | 76 | Quote / unpublished |
| LM011 | [Reacher Technology GB4310 (60T; published speed constant34.4rpm/V)](https://www.reacherbrushless.com/product/product-76-323.html) | 0.2 nominal | 504 loaded at 24 V | 112 | Quote / unpublished |
| LM012 | [Reacher Technology GB5210 (Published speed constant28.25rpm/V)](https://www.reacherbrushless.com/product/product-95-913.html) | 0.46 nominal | 360 loaded at 24 V | 178 | Quote / unpublished |
| LM013 | [SMC Powers PM2205 (standard listed winding; Kv100 rpm/V)](https://shop.smc-powers.com/PM2205.html) | 0.04 nominal | 792 loaded at 12 V | 20 | 14.1 USD |
| LM014 | [SMC Powers PM2805 (standard listed winding; Kv121 rpm/V)](https://shop.smc-powers.com/PM2805NOENCODER.html) | 0.06 nominal | 950 loaded at 12 V | 45 | 14.61 USD |
| LM015 | [SMC Powers PM3505 (standard listed winding; Kv74 rpm/V)](https://shop.smc-powers.com/PM3505-Gimbal-Motor.html) | 0.07 nominal | 587 loaded at 12 V | 42 | 30.09 USD |
| LM016 | [SMC Powers PM3510 (standard listed winding; Kv45 rpm/V)](https://shop.smc-powers.com/PM3510-Gimbal-Motor.html) | 0.11 nominal | 336 loaded at 12 V | 73 | 35.81 USD |
| LM017 | [SMC Powers PM4310 (standard listed winding; Kv33 rpm/V)](https://shop.smc-powers.com/PM4310-Gimbal-Motor.html) | 0.21 nominal | 343 loaded at 16 V | 113 | 33.68 USD |
| LM018 | [SMC Powers PM4315 (standard listed winding; Kv23 rpm/V)](https://shop.smc-powers.com/PM4315-M.html) | 0.28 nominal | 238 loaded at 16 V | 158 | 39.88 USD |
| LM019 | [SMC Powers PM5215 (standard listed winding; Kv14 rpm/V)](https://shop.smc-powers.com/PM5215.html) | 0.76 nominal | 221 loaded at 24 V | 246 | 33.88 USD |
| LM020 | [SMC Powers PM5225 (standard listed winding; Kv14 rpm/V)](https://shop.smc-powers.com/PM5225.html) | 1.26 nominal | 221 loaded at 24 V | 330 | 39.63 USD |
| LM021 | [SMC Powers PM6010 (standard listed winding; Kv11 rpm/V)](https://shop.smc-powers.com/PM6010-Gimbal-Motor.html) | 0.73 nominal | 182 loaded at 24 V | 247 | 37.85 USD |
| LM022 | [SMC Powers PM6025 (standard listed winding; Kv7 rpm/V)](https://shop.smc-powers.com/PM6025.html) | 1.77 nominal | 166 loaded at 36 V | 511 | 42.8 USD |
| LM023 | [T-MOTOR GB36-1 (KV50)](https://store.tmotor.com/product/gb36-1-gimbal-type.html) | 0.24 quoted | Loaded speed unpublished | 88 | 49.9 USD |
| LM024 | [T-MOTOR GB54-2 (KV26)](https://store.tmotor.com/goods-445-GB54-2.html) | 0.45 quoted | Loaded speed unpublished | 156 | 74.9 USD |
| LM025 | [iPower Motors / iFlight GM4108H-120T](https://shop.iflight.com/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217) | 0.1177–0.1765 quoted | 513–567 unloaded at 20 V | 124 | 35.9 USD |
| LM026 | [iPower Motors / iFlight GM5208-12 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-motor-gm5208-12-brushless-gimbal-motor-pro279) | 0.1765–0.2452 quoted | 456–504 unloaded at 20 V | 195 | 39.9 USD |
| LM027 | [iPower Motors / iFlight GM5208-24 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-motor-gm5208-24-brushless-gimbal-motor-pro1347) | 0.1765–0.2452 quoted | 396–436 unloaded at 20 V | 204 | 42.99 USD |
| LM028 | [iPower Motors / iFlight GM6208-150T](https://shop.iflight.com/ipower-motor-gm6208-150t-brushless-gimbal-motor-pro208) | 0.255–0.353 quoted | 209–231 unloaded at 20 V | 249 | 59.9 USD |

## Missing loaded-speed data

| ID | Manufacturer / motor | Torque (Nm; meaning shown) | Speed evidence (rpm) | Mass (g) | Listed price |
|---|---|---|---|---:|---:|
| LM029 | [MAD Motor 4006 EEE V1.0 (320KV)](https://www.madmotor.cn/?page_id=37469&post_type=product) | Unpublished | Loaded speed unpublished | 72 | 299 CNY |
| LM030 | [MAD Motor 5010 IPE V3.0 (200KV)](https://mad-motor.com/products/mad-components-5010-ipe-v3) | Unpublished | Loaded speed unpublished | 181 | 88 USD |
| LM031 | [RCTimer 5010 (260KV)](https://www.rctimer.com/rctimer-5010-260kv-multirotor-brushless-motor40mm-shaft-p0132.html) | Unpublished | Loaded speed unpublished | 100 | 15.99 USD |
| LM032 | [SMC Powers PM2305 (standard; Kv unpublished)](https://shop.smc-powers.com/Motor/For-Gimbal-Motor?order=ASC&page=13&sort=p.model) | 0.04 nominal | Loaded speed unpublished | — | 14.5 USD |
| LM033 | [SMC Powers PM2804 (legacy standard; Kv unpublished)](https://shop.smc-powers.com/Motor/For-Gimbal-Motor?order=ASC&page=13&sort=p.model) | 0.03 quoted | Loaded speed unpublished | 33 | 20.44 USD |
| LM034 | [SMC Powers PM4108 (legacy standard; Kv unpublished)](https://shop.smc-powers.com/PM4108-Gimbal-Motor.html) | 0.15 quoted | Loaded speed unpublished | 85 | 37.46 USD |
| LM035 | [SMC Powers PM5208 (legacy standard; Kv unpublished)](https://shop.smc-powers.com/PM5208-Gimbal-Motor.html) | 0.21 quoted | Loaded speed unpublished | 129 | 49.39 USD |
| LM036 | [SMC Powers PM5210 (standard; Kv unpublished)](https://shop.smc-powers.com/PM5210-Gimbal-Motor.html) | 0.6 quoted | Loaded speed unpublished | 175.4 | 37.98 USD |
| LM037 | [iPower Motors / iFlight GBM2804H-100T](https://shop.iflight.com/ipower-gimbal-brushless-motor-gbm2804h-100t-pro218) | 0.02452–0.03432 quoted | 1468–1622 unloaded at 10 V | 39 | 18.99 USD |
| LM038 | [iPower Motors / iFlight GM2804 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-gm2804-gimbal-motor-pro1153) | 0.02452–0.03432 quoted | 1602–1771 unloaded at 10 V | 42.3 | 21.99 USD |
| LM039 | [iPower Motors / iFlight GM3506 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-motor-gm3506-brushless-gimbal-motor-pro967) | 0.05884–0.09807 quoted | 2149–2375 unloaded at 16 V | 64 | 23.99 USD |

## Outside preferred envelope

| ID | Manufacturer / motor | Torque (Nm; meaning shown) | Speed evidence (rpm) | Mass (g) | Listed price |
|---|---|---|---|---:|---:|
| LM040 | [CubeMars GL60 II (KV28)](https://www.cubemars.com/product/gl60-II-kv28-gimbal-motor.html) | 0.6 rated | 153 loaded at 16 V | 276 | 167 USD |
| LM041 | [CubeMars GL80 (KV30)](https://www.cubemars.com/goods.php?id=1134) | 1 rated | 450 loaded at 24 V | 312 | Quote / unpublished |
| LM042 | [CubeMars GL80 (KV60)](https://www.cubemars.com/goods.php?id=1134) | 1 rated | 1010 loaded at 24 V | 315 | Quote / unpublished |
| LM043 | [iPower Motors / iFlight GM110-10 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-motor-gm110-10-brushless-gimbal-motor-pro1297) | 0.8826–1.471 quoted | 292–323 unloaded at 20 V | 532 | 190 USD |
| LM044 | [iPower Motors / iFlight GM110-8 (stock listing; winding turns not separately published)](https://shop.iflight.com/ipower-motor-gm110-8-brushless-gimbal-motor-pro197) | 0.7845–1.177 quoted | 125–139 unloaded at 20 V | 534 | 170 USD |
| LM045 | [iPower Motors / iFlight GM8112-80T](https://shop.iflight.com/ipower-motor-gm8112-brushless-gimbal-motor-pro195) | 0.5884–0.8826 quoted | 380–420 unloaded at 22 V | 428 | 129.9 USD |

## Record conditions and source differences

### LM001 — CubeMars GL30

Published rated speed exceeds1500rpm; lightweight wrist research candidate, not a full actuator qualification. USD50.99 slightly exceeds former USD50 budget.

Source differences: Marketed KV290 versus detailed Kv255rpm/V. Store gives rated2200rpm/2.3A; detailed current table1990rpm/2.13ADC.

Primary source: [GL30](https://www.cubemars.com/product/gl30-kv290-gimbal-motor.html).

### LM002 — CubeMars GL40

Published.25Nm/2100rpm at16V is a credible lightweight1500rpm research point. USD72.99 above former budget; driver not included.

Source differences: Marketed KV210 versus detailed Kv195rpm/V; generic prose peak.73Nm versus winding-specific.75Nm.

Primary source: [GL40](https://www.cubemars.com/product/gl40-kv210-gimbal-motor.html).

### LM003 — CubeMars GL40 II

Nominal.25Nm/697rpm at16V. Separate official24V operating graph visually inspected in prior review indicates approximately.12Nm at1500rpm; a curve point, not a continuous thermal rating. Integrated driver differs from user's own FOC preference.

Source differences: Earlier prose/PDF112g versus current detailed/comparison125g. Older PDF inertia.794578gcm2 versus current79.45gcm2, factor100 discrepancy needs manufacturer confirmation. 16V graph and no-load-speed table differ; no combined map invented.

Primary source: [GL40 II](https://www.cubemars.com/product/gl40-II-kv82-5-gimbal-motor.html).

### LM004 — CubeMars GL35

Rated and no-load speeds at16V below1500rpm. Need approved higher-voltage/different-winding map or slower target. Bare price exceedsUSD50.

Source differences: Marketed KV100 versus detailed Kv82.5rpm/V. Store rated900rpm versus detailed815rpm. Comparison table resistance23600mOhm conflicts with detailed3600mOhm.

Primary source: [GL35](https://www.cubemars.com/product/gl35-kv100-gimbal-motor.html).

### LM005 — CubeMars GL40

KV70 at16V is too slow for1500rpm even unloaded; cannot transfer KV210 winding speed rating.

Source differences: Marketed KV70 versus detailed Kv63.5rpm/V; generic prose peak.73Nm versus winding-specific.50Nm.

Primary source: [GL40](https://www.cubemars.com/product/gl40-kv70-gimbal-motor.html).

### LM006 — CubeMars GL60

Published rated and no-load speed at24V below1500rpm. Higher voltage requires approved torque/speed/thermal map. Separate winding facts retained.

Primary source: [GL60](https://www.cubemars.com/goods.php?id=1133).

### LM007 — CubeMars GL60

Published rated and no-load speed at24V below1500rpm. Higher voltage requires approved torque/speed/thermal map. Separate winding facts retained.

Source differences: Specific current KV55 live offer/price not established; official catalog facts retained.

Primary source: [GL60](https://www.cubemars.com/images/file/20230703/1688352405877552.pdf).

### LM008 — Huizhou Techno Motor Co., Ltd. (TechnoMotor / TejiaMotor product sites) GBM3506H-130T

Additional manufacturer-distinct GBM research candidate. Published winding Kv and supply range suggest insufficient speed for the 1500rpm geared application; seek a faster winding. Only stall torque is published, with no continuous thermal duty or simultaneous torque-speed operating point. Maximum-current/power labels are not treated as continuous servo ratings.

Source differences: Brand/OEM relationship to legacy iPower GBM models is unverified; do not merge matching model names across manufacturers.

Primary source: [GBM3506H-130T](https://www.tejiamotor.com/Pr_d_gci_112_id_123.html).

### LM009 — Huizhou Techno Motor Co., Ltd. (TechnoMotor / TejiaMotor product sites) GBM5208H-200T

Additional manufacturer-distinct GBM research candidate. Published winding Kv and supply range suggest insufficient speed for the 1500rpm geared application; seek a faster winding. Only stall torque is published, with no continuous thermal duty or simultaneous torque-speed operating point. Maximum-current/power labels are not treated as continuous servo ratings.

Source differences: Brand/OEM relationship to legacy iPower GBM models is unverified; do not merge matching model names across manufacturers.

Primary source: [GBM5208H-200T](https://www.technomotor.com/En_Pr_d_gci_112_id_128.html).

### LM010 — Reacher Technology GB3510

No-encoder mass76g; encoder version89g. Published maximum speed965rpm is not described as no-load speed, so no_load_speed_rpm remains null. Stock12V point does not satisfy1500rpm. Manufacturer offers custom voltage/speed/windings in its motor portfolio; a new approved operating point would be required. Stall torque has no published duration. OEM relationship to similarly named GB motors sold elsewhere is unverified; this row is one supplier listing, not proof of independent electromagnetic design.

Primary source: [GB3510](https://www.reacherbrushless.com/product/product-73-87.html).

### LM011 — Reacher Technology GB4310

No-encoder mass112g; encoder version128g. Published maximum speed1028rpm is not explicitly a no-load value, so no_load_speed_rpm remains null. Stock24V point fails1500rpm. A faster factory winding or manufacturer-approved different bus point must be specified and tested; no extrapolated voltage/torque rating is imported. Stall torque duration is not supplied. OEM identity versus similarly named GB motors is unverified.

Primary source: [GB4310](https://www.reacherbrushless.com/product/product-76-323.html).

### LM012 — Reacher Technology GB5210

No-encoder mass178g; encoder version202g. Published maximum speed678rpm is not explicitly no-load speed. The .46Nm nominal rating is at360rpm, not1500rpm; do not enter it as a passed geared-arm candidate. A faster approved winding may be interesting at this size. Stall torque1.03Nm has no duration. Supplier/OEM independence from other GB listings is unverified.

Source differences: Headline says 'Number of pole pairs:11 poles'; detailed table specifies11 pairs. Pole count is not imported into this schema.

Primary source: [GB5210](https://www.reacherbrushless.com/product/product-95-913.html).

### LM013 — SMC Powers PM2205

Nominal 0.04Nm at792rpm; published Max Speed 1200rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Primary source: [PM2205](https://shop.smc-powers.com/PM2205.html).

### LM014 — SMC Powers PM2805

Nominal 0.06Nm at950rpm; published Max Speed 1452rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Individual body table gives44g/.03Nm at11.1V; shared nominal table gives45g/.06Nm at12V. Older cross-model table gives33.2x15mm and inconsistent slot/pole labels; individual table gives33.5x15.3mm and24N28P, shared electrical table7 pole pairs.

Primary source: [PM2805](https://shop.smc-powers.com/PM2805NOENCODER.html).

### LM015 — SMC Powers PM3505

Nominal 0.07Nm at587rpm; published Max Speed 890rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Main bare-page body table gives58g; detailed shared nominal table/other pages give42g. Legacy torque headline .06Nm at11.1V versus nominal .07Nm at12V. Separate out-of-stock PM3505-GMotor.html code25350508 priceUSD2.63 is not treated as a current equivalent bare-motor offer.

Primary source: [PM3505](https://shop.smc-powers.com/PM3505-Gimbal-Motor.html).

### LM016 — SMC Powers PM3510

Nominal 0.11Nm at336rpm; published Max Speed 550rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Legacy headline .10Nm at11.1V versus nominal .11Nm at12V.

Primary source: [PM3510](https://shop.smc-powers.com/PM3510-Gimbal-Motor.html).

### LM017 — SMC Powers PM4310

Nominal 0.21Nm at343rpm; published Max Speed 520rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Legacy body/torque table120g/.19Nm at14.8V versus detailed nominal113g/.21Nm at16V.

Primary source: [PM4310](https://shop.smc-powers.com/PM4310-Gimbal-Motor.html).

### LM018 — SMC Powers PM4315

Nominal 0.28Nm at238rpm; published Max Speed 360rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Primary source: [PM4315](https://shop.smc-powers.com/PM4315-M.html).

### LM019 — SMC Powers PM5215

Nominal 0.76Nm at221rpm; published Max Speed 336rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Official external drawing dimensions not reliably extracted; similar Steadywin/LKM reseller dimensions were not substituted.

Primary source: [PM5215](https://shop.smc-powers.com/PM5215.html).

### LM020 — SMC Powers PM5225

Nominal 1.26Nm at221rpm; published Max Speed 336rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Official external drawing dimensions not reliably extracted; similar Steadywin/LKM reseller dimensions were not substituted.

Primary source: [PM5225](https://shop.smc-powers.com/PM5225.html).

### LM021 — SMC Powers PM6010

Nominal 0.73Nm at182rpm; published Max Speed 275rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Legacy headline .80Nm at14.8V versus nominal .73Nm at24V.

Primary source: [PM6010](https://shop.smc-powers.com/PM6010-Gimbal-Motor.html).

### LM022 — SMC Powers PM6025

Nominal 1.77Nm at166rpm; published Max Speed 252rpm. Stock winding is not a verified1500rpm choice. Requires a supplier-approved voltage/winding change or slower joint target. Null external dimensions remain unverified.

Source differences: Official external drawing dimensions not reliably extracted; similar Steadywin/LKM reseller dimensions were not substituted.

Primary source: [PM6025](https://shop.smc-powers.com/PM6025.html).

### LM023 — T-MOTOR GB36-1

Stock KV50 winding is specified for 3-6S LiPo. Published shaft bore5.6mm and internal resistance15.6ohm. No published loaded1500rpm point, rated-speed table, rotor inertia, or timed peak limit. Even ideal KV times fully charged6S25.2V is1260rpm, so this winding is not a1500rpm candidate at its listed supply. Factory faster winding would need a new torque/speed/thermal specification.

Primary source: [GB36-1](https://store.tmotor.com/product/gb36-1-gimbal-type.html).

### LM024 — T-MOTOR GB54-2

Stock KV26 winding is specified for3-6S LiPo, with12.7mm bore and15ohm internal resistance. Nominal torsion is not a verified continuous torque at1500rpm. Ideal KV times fully charged6S gives only655rpm; a faster approved winding is required. Price exceeds the preferred USD50 motor budget.

Source differences: Product page subheading incorrectly names GB36-1 while title, specification and pack list name GB54-2.

Primary source: [GB54-2](https://store.tmotor.com/goods-445-GB54-2.html).

### LM025 — iPower Motors / iFlight GM4108H-120T

Compact torque candidate only with a different winding or explicitly qualified operating point. Stock no-load maximum567rpm at20V is far below1500rpm, despite quoted0.1177–0.1765Nm load torque.

Encoder options: AS5048A magnetic absolute encoder,14bit, 137 g, 54.99 USD

Primary source: [GM4108H-120T](https://shop.iflight.com/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217).

### LM026 — iPower Motors / iFlight GM5208-12

Stock motor has useful quoted0.1765–0.2452Nm torque but only456–504rpm no-load at20V. Qualify a higher-speed winding/voltage before considering the geared1500rpm application.

Encoder options: AS5048A magnetic absolute encoder,14bit, 209 g, 61.99 USD

Primary source: [GM5208-12](https://shop.iflight.com/ipower-motor-gm5208-12-brushless-gimbal-motor-pro279).

### LM027 — iPower Motors / iFlight GM5208-24

Stock no-load396–436rpm at20V fails1500rpm target. Physically distinct24N/22P version rather than assuming it is an electrical duplicate of the12N/14P GM5208-12. Category price USD42.99; one product snapshot displayed a USD37.83 promotion, so checkout pricing may vary.

Primary source: [GM5208-24](https://shop.iflight.com/ipower-motor-gm5208-24-brushless-gimbal-motor-pro1347).

### LM028 — iPower Motors / iFlight GM6208-150T

At size/mass boundary and aboveUSD50 preference. Stock no-load209–231rpm at20V is unsuitable for1500rpm; included only as a rewinding/manufacturer-inquiry candidate.

Source differences: Bare listing's24mm height text incorrectly includes '(Included with Encoder housing)' while encoder model is32mm/265g. Use bare values with mechanical drawing confirmation.

Encoder options: AS5048A magnetic absolute encoder,14bit, 265 g, 81.99 USD

Primary source: [GM6208-150T](https://shop.iflight.com/ipower-motor-gm6208-150t-brushless-gimbal-motor-pro208).

### LM029 — MAD Motor 4006 EEE V1.0

Prototype/custom-winding sourcing pool only. Manufacturer lists320/380/740KV and6S-8S use, but does not provide a continuous servo torque at1500rpm or still-air holding thermal rating. Drone propeller thrust and maximum power are not motor-joint torque ratings. Factory explicitly offers customKV/windings, OEM/ODM and test reports; request the required torque-speed-temperature map before selection. No numeric torque is copied from retailer claims.

Source differences: Chinese domestic and international storefront prices refer to different markets; no currency conversion or landed-cost equivalence is assumed.

Primary source: [4006 EEE V1.0](https://www.madmotor.cn/?page_id=37469&post_type=product).

### LM030 — MAD Motor 5010 IPE V3.0

Larger prototype/custom-winding sourcing pool.200KV version lists12S support and181g;240/310KV variants have different177/176g masses and are not merged here. Published696ZZx2 bearings and6mm internalshaft. Thrust/propeller tests do not establish continuous still-air torque at1500rpm; no servo torque or rotor inertia rating is imported. Manufacturer custom winding inquiry may produce a better geared-arm operating point, but no quotation or approval is claimed.

Primary source: [5010 IPE V3.0](https://mad-motor.com/products/mad-components-5010-ipe-v3).

### LM031 — RCTimer 5010

Prototype pool only. Manufacturer lists4-6S LiPo,12N14P,4mm external/5mm internal shaft,0.4A no-load current at10V,20A and450W explicitly for180seconds. No continuous servo torque, rated shaft-speed/torque pair, rotor inertia or still-air thermal map is published. Published0.6ohm internal resistance lacks measurement convention; current/power ratings must not be treated as an indefinite joint rating.36V exceeds the listed4-6S battery range. High nominal watts do not mean the motor draws that power at all operating points.

Source differences: One listing combines260/360/530/620KV variants with different weights/envelopes; this row uses only the260KV100g50x26mm block.

Primary source: [5010](https://www.rctimer.com/rctimer-5010-260kv-multirotor-brushless-motor40mm-shaft-p0132.html).

### LM032 — SMC Powers PM2305

Official listing gives5mm shaft hole,.04Nm and maximum.06Nm. No reliable speed, body envelope or mass; PM2205 data not borrowed.

Primary source: [PM2305](https://shop.smc-powers.com/Motor/For-Gimbal-Motor?order=ASC&page=13&sort=p.model).

### LM033 — SMC Powers PM2804

Legacy28x4 stator unit; input8-12V, no verified loaded speed or rotor inertia.

Source differences: Individual prose33.2x16mm versus shared table33.2x15mm; some older PM2804/PM2805 cross-model headers inconsistent.

Primary source: [PM2804](https://shop.smc-powers.com/Motor/For-Gimbal-Motor?order=ASC&page=13&sort=p.model).

### LM034 — SMC Powers PM4108

Out of stock when checked; input12-24V, loaded speed and inertia unknown.

Source differences: Prose rounds length27mm; body table27.1mm.

Primary source: [PM4108](https://shop.smc-powers.com/PM4108-Gimbal-Motor.html).

### LM035 — SMC Powers PM5208

Out of stock; official page says updated toPM5210. Retained legacy lead, not verified1500rpm candidate.

Primary source: [PM5208](https://shop.smc-powers.com/PM5208-Gimbal-Motor.html).

### LM036 — SMC Powers PM5210

Replacement forPM5208; body/torque table available, no nominal loaded speed or rotor inertia found. Cannot qualify1500rpm.

Primary source: [PM5210](https://shop.smc-powers.com/PM5210-Gimbal-Motor.html).

### LM037 — iPower Motors / iFlight GBM2804H-100T

Very light inexpensive J6 or low-load wrist research candidate. Published no-load range straddles 1500 rpm; quoted load torque is only 0.0245–0.0343 Nm and no loaded speed is provided. Not qualified for 1500 rpm under torque.

Primary source: [GBM2804H-100T](https://shop.iflight.com/ipower-gimbal-brushless-motor-gbm2804h-100t-pro218).

### LM038 — iPower Motors / iFlight GM2804

Inexpensive low-load/J6 candidate; no-load speed is near target and there is no torque-speed or continuous-duty evidence at 1500 rpm. Category price USD21.99; one product snapshot displayed a USD19.35 promotion, so checkout pricing may vary.

Encoder options: AS5048A magnetic absolute encoder,14bit, 51 g, 38.99 USD

Primary source: [GM2804](https://shop.iflight.com/ipower-gm2804-gimbal-motor-pro1153).

### LM039 — iPower Motors / iFlight GM3506

Most relevant GM candidate for lightweight wrists: 64 g, quoted 0.0588–0.0981 Nm load torque and no-load2149–2375 rpm at16V. No simultaneous loaded-speed/torque point or thermal duration is published. Verify 1500rpm torque and permitted voltage; the stated 2–3S working range conflicts with the 16V no-load test.

Source differences: Product lists a16V no-load test but2–3S working supply. Rated permissible operating voltage and loaded1500rpm behavior require manufacturer confirmation.

Encoder options: AS5048A magnetic absolute encoder,14bit, 72 g, 41.9 USD

Primary source: [GM3506](https://shop.iflight.com/ipower-motor-gm3506-brushless-gimbal-motor-pro967).

### LM040 — CubeMars GL60 II

Near-boundary:70.5mm exceeds70mm by.5mm. Integrated driver. Additional24V point.6Nm/319rpm,1.5Nm peak remains well below1500rpm.

Source differences: Comparison table28 pole pairs conflicts with detailed24N28P=14 pole pairs. Some official summary prose279g versus detailed276g.

Primary source: [GL60 II](https://www.cubemars.com/product/gl60-II-kv28-gimbal-motor.html).

### LM041 — CubeMars GL80

87mm exceeds70mm envelope. Coverage reference only; rated/no-load speed at24V below1500rpm.

Source differences: GL80 generic heading315g; winding-specific comparison table KV30 312g, KV60 315g.

Primary source: [GL80](https://www.cubemars.com/goods.php?id=1134).

### LM042 — CubeMars GL80

87mm exceeds70mm envelope. Coverage reference only; rated/no-load speed at24V below1500rpm.

Source differences: GL80 generic heading315g; winding-specific comparison table KV30 312g, KV60 315g.

Primary source: [GL80](https://www.cubemars.com/goods.php?id=1134).

### LM043 — iPower Motors / iFlight GM110-10

Outside diameter/mass/budget envelope and slow; appendix only.

Encoder options: AS5048A magnetic absolute encoder,14bit, 556 g, 212 USD

Primary source: [GM110-10](https://shop.iflight.com/ipower-motor-gm110-10-brushless-gimbal-motor-pro1297).

### LM044 — iPower Motors / iFlight GM110-8

Outside diameter/mass/budget envelope and slow; appendix only.

Encoder options: AS5048A magnetic absolute encoder,14bit, 610 g, 192 USD

Primary source: [GM110-8](https://shop.iflight.com/ipower-motor-gm110-8-brushless-gimbal-motor-pro197).

### LM045 — iPower Motors / iFlight GM8112-80T

Outside preferred70mm diameter and250g mass envelope, above budget, and slow stock winding. Excluded from active fit shortlist; retained for category coverage.

Source differences: Bare listing's26mm height text incorrectly says included encoder housing; encoder model is33.4mm/446g.

Encoder options: AS5048A magnetic absolute encoder,14bit, 446 g, 152 USD

Primary source: [GM8112-80T](https://shop.iflight.com/ipower-motor-gm8112-brushless-gimbal-motor-pro195).

## Research scope

The retained supplier JSON files describe category coverage, individual measurement conditions, encoder options and source disagreements. Bare frameless stator/rotor sets and actuators with an already integrated reducer were excluded from this motor-only search. Oversize family members are retained in the outside-envelope appendix. Propeller motors with only burst/airflow specifications remain prototype candidates with missing servo-duty data.

This research does not replace the active motor.csv or change any saved simulation result. Candidate motor masses and real rotor inertias must be introduced into a new geometry study before selecting final joint hardware.
