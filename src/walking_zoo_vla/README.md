# walking_zoo_vla

This package is a lightweight semantic action layer for future VLA integration.
It does not include a VLA model, ML runtime, or dataset dependency.

The intended flow is:

```text
VLA / LLM agent -> SemanticAction -> walking_zoo runtime or Nav2 -> safety pipeline -> adapter
```

VLA systems should not directly command joints, vendor SDKs, or robot motion.
