# Lingxi X2 Ultra 12-DoF RL-to-Deployment

This private repository records the X2 Ultra 12-DoF reinforcement-learning, policy-export, and MuJoCo sim2sim deployment workflow.

The baseline commit is an unmodified snapshot of the current files. Subsequent commits will contain one validated change at a time, with the reason and evidence recorded in docs/CHANGELOG.md.

## Layout

- training/: IsaacLab X2 task/robot snapshots and existing RL/export scripts.
- deployment/: unmodified legacy sim2sim runner.
- assets/mujoco/: MuJoCo scene and leg model.
- assets/usd/: small USD reference/configuration files.
- models/: exported policy and one resume checkpoint.
- materials/: run configuration, diagnostics, and reproducibility notes.
- docs/: setup and change history.

IsaacLab/Isaac Sim and the existing humanoid training package remain external prerequisites.
