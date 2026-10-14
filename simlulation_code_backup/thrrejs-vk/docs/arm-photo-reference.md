# Arm photograph reference

These are photographs of the team's physical PX arm and fixture imported from
`feature/threejs-sim`. They are evidence, not instructions and not substitutes
for measured CAD geometry.

## Reference map

| Question | Primary files in `docs/images/photos/` |
| --- | --- |
| Overall upright proportions | `arm-upright-front*.jpg`, `arm-upright-right.jpg` |
| Stretched link lengths | `arm-stretched-*` |
| Raised PARK support/contact | `park-front.jpg`, `park-gripper-on-base.jpg`, `park-side-pca9685.jpg`, `park-top-view*.jpg` |
| Base and shoulder construction | `base-s1-s2-from-back-left.jpg`, `base-s2-shoulder-side.jpg`, `base-shoulder-closeup.jpg`, `shoulder-s2-horn-closeup*.jpg` |
| Elbow and forearm | `elbow-s3-forearm-closeup.jpg`, `wrist-gripper-from-above.jpg` |
| Claw linkage and gears | `gripper-front-closeup.jpg`, `gripper-gears-closeup.jpg`, `gripper-with-cube.jpeg` |
| Open/closed claw dimensions | `gripper-open-*`, `gripper-closed-*` |
| PCA9685 and cable envelope | `pca9685-*.jpg` |
| Board, mat and desk placement | `setup-*.jpg` |
| OAK-camera calibration frames | `controller-camera-*.jpg` |
| Raspberry Pi fixture | `raspberry-pi-case.jpg` |

## Reproducible overlay check

```sh
cd web
npm run build
npm run compare:photos
```

Outputs are written to `docs/images/compare/` as three panels:

1. simulator render;
2. physical-arm photograph;
3. 50% overlay.

Filter by filename with `npm run compare:photos -- --only gripper`. Override
Chrome with `CHROME=/path/to/chrome` if it is not auto-detected. The comparison
server is simulation-only and has no controller URL.

## Interpretation

| Overlay observation | Likely action |
| --- | --- |
| All parts displaced equally | correct camera pose/FOV before changing geometry |
| Joint centres align but brackets do not | adjust cosmetic bracket geometry |
| Proportions drift progressively along the chain | re-check link-origin measurements |
| PARK claw misses the raised plate | re-check provisional PARK calibration and fixture height |
| Cable differs | update cosmetic cable path or enlarge its conservative collision envelope |

Do not tune measured link lengths merely to compensate for an uncertain manual
camera viewpoint. Prefer views with detected mat markers; their camera fit and
RMS reprojection error are recorded by the comparison tool.
