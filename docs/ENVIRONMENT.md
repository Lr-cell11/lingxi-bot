# Environment and path notes

Required external software:
- IsaacLab and Isaac Sim for training/export.
- MuJoCo Python environment for sim2sim.
- PyTorch, ONNX Runtime, and packages required by the existing training tree.

The baseline keeps the original absolute USD path in agibot_x2.py because this commit is intentionally unmodified. A later portability commit will replace it with a configurable path.

The two *_base.usd files are each about 113 MB and exceed GitHub's normal 100 MB blob limit. They are not included in the baseline; materials/ASSET_MANIFEST.md records their paths. Use Git LFS or an artifact store before claiming a fully self-contained Isaac Sim checkout.
