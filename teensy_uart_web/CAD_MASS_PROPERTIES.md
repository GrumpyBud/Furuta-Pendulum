# CAD mass and material analysis

This analysis uses `/home/otharp/Downloads/Main Assembly.zip` and the later
single assembled `/home/otharp/Downloads/Main Assembly.step`, supplied on
2026-08-04. Both contain 28 solid bodies. The assembled STEP is authoritative
for component placement and axis orientation; the individual bodies remain
useful for volume identification. Base-frame members are intentionally
excluded. Dimensions and volumes were read with Open CASCADE through CadQuery;
user measurements override known-stale CAD dimensions.

## Geometry corrections and coordinate status

- The STEP motor-arm 2020 extrusion is `280 mm`; the real part is `140 mm`.
- The STEP pendulum rod is `120 mm`; the real part is `200 mm`.
- The STEP horizontal pendulum-pivot shaft is `8 mm x 78 mm` and is reported
  accurate.
- The printed-body geometry is reported accurate.
- In the assembled STEP, the vertical motor axis is at `(x, y) = (0, 0)`, the
  280 mm arm runs radially along the Y axis, and the pendulum rod's pivot/
  attachment centre is at `y = -321.5 mm`. Holding every other CAD offset fixed
  and shortening the extrusion to the real `140 mm` gives
  `321.5 - (280 - 140) = 181.5 mm`.
- The assembled STEP directly confirms that the 78 mm horizontal pendulum shaft
  is parallel to the radial 2020 arm, not perpendicular to it.

## Material assignments

| Component family | Assigned material | Density used |
|---|---|---:|
| Printed top components | Bambu PETG | `1.25 g/cm^3` solid polymer |
| 2020 extrusion and flange hub | Aluminum | `2.70 g/cm^3` |
| 8 mm shafts | 304 stainless steel | `8.00 g/cm^3` |
| Bearings and ordinary fasteners | Steel | `7.85 g/cm^3` |

Printed parts are not solid even though STEP volume is solid. The nominal
estimate uses 55% of solid-CAD mass, with 40–70% carried as an uncertainty
range. This represents walls, top/bottom layers, and infill without pretending
that an unknown slicer profile is exact.

## STEP component results

| Role | STEP body | CAD volume (cm^3) | Material | Estimated mass |
|---|---|---:|---|---:|
| Motor-end rotating adapter | `Part 1 (1)` | `22.018` | PETG | `15.14 g` nominal (`11.01–19.27 g`) |
| Pendulum 90-degree bracket | `Part 1 (4)` | `15.604` | PETG | `10.73 g` nominal (`7.80–13.65 g`); already covered by measured pendulum mass |
| Outer bearing support | `Part 1 (5)` | `8.997` | PETG | `6.18 g` nominal (`4.50–7.87 g`) |
| Outer bearing/encoder support | `Part 1 (6)` | `41.372` | PETG | `28.44 g` nominal (`20.69–36.20 g`) |
| Aluminum motor hub | `Open CASCADE ... 1.1` | `6.182` | Aluminum | `16.69 g` |
| Hub fasteners, two bodies | `Open CASCADE ... 1.2` | `0.936` total | Steel | `7.35 g` total |
| F688ZZ bearings, two | named bearing bodies | `1.450` total | Steel | `11.38 g` total |
| Horizontal pivot shaft | `Part 15` | `3.921` | 304 stainless | `31.37 g`; already covered by measured pendulum mass |
| Real 140 mm 2020 rotating arm | `Part 12` profile, corrected length | `25.640` | Aluminum | `69.23 g` |

The STEP 2020 cross-sectional material area is `183.14 mm^2`, which is why the
real 140 mm part is estimated directly from CAD profile volume instead of from
a generic extrusion catalogue.

## Motor-side rigid assembly estimate

The motor-side group contains the 140 mm 2020 arm, motor-end PETG adapter,
aluminum hub, the two PETG bearing/encoder supports, two bearings, and hub
fasteners. The encoder board/wiring and unmodelled ordinary screws receive
allowances of `5 g` and `6 g`, respectively.

| Case | Estimated mass |
|---|---:|
| 40% effective PETG density, raw CAD hardware | `151.8 g` |
| 55% nominal effective PETG density, raw CAD hardware | `165.4 g` |
| 70% effective PETG density, raw CAD hardware | `179.0 g` |
| Nominal model allocation after removing one bearing-equivalent | `159.7 g` (`146.2–173.3 g`) |

This mass excludes the fixed motor stator and the measured `159 g` pendulum-side
lump. The D5065 rotor inertia must eventually be included in motor-axis inertia;
ODrive publishes electrical and torque data but not rotor inertia.

## Corrected upright yaw inertia

The single assembled STEP makes it possible to translate each complete outer
body inward by `140 mm`, rather than placing all outer mass at the pendulum
attachment radius. The shortened 2020 keeps its CAD motor end at `y = 12.7 mm`,
ends at `y = -127.3 mm`, and has its COM at `y = -57.3 mm`. Its corrected yaw
inertia is `0.000343 kg m^2`; the earlier `0.00074 kg m^2` estimate was too high.

Representative corrected contributions about the vertical motor axis are:

```text
140 mm 2020 extrusion                         0.000343 kg m^2
two printed bearing/encoder supports          0.000489 kg m^2
two bearings                                  0.000170 kg m^2
78 mm horizontal shaft                        0.000544 kg m^2
printed pendulum bracket                       0.000335 kg m^2
real 200 mm vertical rod at 181.5 mm radius   0.002650 kg m^2
two CAD aluminum hubs and fasteners            0.000625 kg m^2
motor-end printed adapter                      0.000004 kg m^2
```

Those modeled contributions total about `0.00516 kg m^2`. Adding the measured
mass remainder for collars/set screws, the encoder/wiring allowance, unmodelled
mounting screws, and an estimated D5065 rotor contribution gives a nominal
upright generalized yaw inertia of `0.00561 kg m^2`. The controller carries
`0.00510–0.00620 kg m^2` as an engineering uncertainty interval. This is the
complete linearized `M(0,0)` inertia with the pendulum upright, not an arm-only
inertia to which `m*r^2` should be added again.

## Pivot-axis orientation verification

The assembled STEP places both the 2020 arm and the 78 mm horizontal pendulum
shaft along the Y axis. The shaft is therefore radial, as required for the
pendulum to swing in the tangential/vertical plane. An earlier warning that the
two were perpendicular was caused by misreading the separately exported body
geometry and is superseded by this assembled-file check.

## Model bookkeeping

The measured `159 g` pendulum-side lump includes the rod, printed 90-degree
bracket, horizontal shaft, collars, set screws, and one complete bearing as an
approximation of the two bearing assemblies. The motor-side CAD table includes
both complete bearings. When constructing the final dynamic model, the one
bearing-equivalent already allocated to the pendulum-side pivot lump must be
subtracted or otherwise kept out of the arm component sum. Mass on the pivot
axis can be assigned to either generalized body, but it must never be counted
twice.
