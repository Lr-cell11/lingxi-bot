# Validated deployment context

- Policy observation width: 48.
- Action target: default_pos + 0.5 * action.
- PD: stiffness 30.0, damping 0.5.
- MuJoCo timestep: 0.005 s.
- Isaac policy step: 0.005 s physics step with decimation 4 (0.02 s policy step).
- DCMotor effort limits: 120 Nm for hips/knees, 60 Nm ankle pitch, 36 Nm ankle roll.
- DCMotor saturation effort: 120 Nm.
- Initial MuJoCo/ONNX and Isaac actions were aligned to approximately 1e-6.
- Remaining work is contact/dynamics alignment, runner portability, and deterministic validation; retraining is not part of this baseline.
