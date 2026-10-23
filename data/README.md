# data 目录

- `videos/v1_green_target.mp4`、`videos/v2_green_target.mp4`
  - 用户提供的两段原始视频（同一绿色发光目标，720x1280 竖屏，30fps）。
  - 来源：`F:\微信重要文件存储\xwechat_files\wxid_6nso9ojijeh222_5af8\msg\video\2026-10\`
    （`d8f480dd6df50fb5b649c39c1f41edca.mp4`、`3078f20eebc857f33a4b0b7cbb2f5a3d.mp4`）。
  - 仅用于算法验证，未做任何修改。
- `recon/`
  - 开发前侦察产物：宽阈值绿色掩膜逐帧统计 CSV、原始帧与目标放大图。
  - 结论见 `docs/recon_target_analysis.md`。
- `../results/qvga/`（运行时生成，已被 .gitignore 忽略）
  - 由两个原始视频转制的 QVGA 320x240 部署分辨率模拟视频（旋转 90° → 320x180 → letterbox），
    用于验证 OpenMV 相机分辨率下的检测/PnP/滤波（见 `docs/validation_report.md` §5）。
- `synthetic/`（运行时生成，已被 .gitignore 忽略）
  - `glowtrack_synth` 生成的合成图像序列与真值 CSV。
- `calibration/`（运行时生成）
  - 棋盘格标定图像与输出相机内参。
  - 其中 `chess_synth/` 由 `_recon/gen_chessboard.py` 生成的合成棋盘格（640x480，真实
    fx=500、畸变已给定），用于回归验证 `glowtrack --mode calibrate`：
    生成的 18 张图可标定出 fx≈505、RMS≈0.29 px（见 `docs/calibration.md`）。
