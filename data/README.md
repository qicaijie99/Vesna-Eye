# data 目录

- `videos/v1_green_target.mp4`、`videos/v2_green_target.mp4`
  - 用户提供的两段原始视频（同一绿色发光目标，720x1280 竖屏，30fps）。
  - 来源：`F:\微信重要文件存储\xwechat_files\wxid_6nso9ojijeh222_5af8\msg\video\2026-10\`
    （`d8f480dd6df50fb5b649c39c1f41edca.mp4`、`3078f20eebc857f33a4b0b7cbb2f5a3d.mp4`）。
  - 仅用于算法验证，未做任何修改。
- `recon/`
  - 开发前侦察产物：宽阈值绿色掩膜逐帧统计 CSV、原始帧与目标放大图。
  - 结论见 `docs/recon_target_analysis.md`。
- `synthetic/`（运行时生成，已被 .gitignore 忽略）
  - `glowtrack_synth` 生成的合成图像序列与真值 CSV。
- `calibration/`（运行时生成）
  - 棋盘格标定图像与输出相机内参。
