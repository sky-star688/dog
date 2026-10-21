# 模型目录

把 Windows 训练后导出的 `best.onnx` 放在此目录，并保证 `labels.txt` 的类别顺序与训练用 `data.yaml` 中的 `names` 完全一致。

默认类别为：

1. `anomaly`：需要报警但不一定抓取的异常。
2. `block`：需要机械臂抓取的物块。

如果使用其他类别，请同时修改 `src/lite3_inspection/config/inspection.yaml` 中的 `target_classes`。
