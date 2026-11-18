# Intrinsic Core VI — OMTS 3D Demo

Mô phỏng 3D bằng trình duyệt, minh họa chu trình robot phục vụ máy CNC theo ý tưởng từ [Intrinsic OMTS](https://github.com/intrinsic-ai/intrinsic-omts).

**Demo online (sau khi GitHub Pages được bật và triển khai thành công):** https://xulytiengviet.github.io/intrinsic-core/

## Chạy thử và triển khai

- Trên GitHub: vào **Settings → Pages** của repository `xulytiengviet/intrinsic-core`, chọn **Build and deployment → Source: GitHub Actions**. Workflow `.github/workflows/deploy-3d-demo.yml` tự triển khai thư mục `docs/` khi push lên `main`; có thể chạy thủ công qua **Actions → Deploy OMTS 3D demo**.
- Chạy cục bộ: từ thư mục gốc, thực hiện `python3 -m http.server 8000 --directory docs` rồi mở `http://localhost:8000`.
- Trình duyệt phải hỗ trợ WebGL và tải được thư viện `three.js` từ jsDelivr. Không cần ROS 2, Gazebo, GPU NVIDIA hay server nội bộ để xem demo.

## Mô phỏng 8 bước

| Bước | Hoạt động hiển thị | Gợi nhớ khái niệm OMTS |
| --- | --- | --- |
| 1 | Camera quét phôi | RGB-D capture |
| 2 | Hiển thị hộp 6-DoF và tọa độ phôi | `estimate_pose` |
| 3 | Đường màu xanh qua waypoint lập sẵn | `move_robot`, planning |
| 4 | Robot hạ tay, gắp phôi, nâng lên | `move_to_contact` / gripper |
| 5 | Cửa CNC mở, nạp phôi, kẹp ê-tô | load machine + digital I/O |
| 6 | Cửa đóng, spindle giả lập chạy | machining handshake |
| 7 | Robot mở CNC và lấy sản phẩm | unload machine |
| 8 | Đặt vào khay thành phẩm và về home | return infeed |

**Điều khiển:** Chạy/Tạm dừng, Đặt lại, Bước kế tiếp, tốc độ 0.5–3×, lặp chu trình, bật/tắt lớp quỹ đạo và quan sát 6D; đổi góc tổng quan/trên cao/CNC; dùng chuột để xoay và phóng to.

## Giới hạn kỹ thuật và an toàn

- Các đối tượng 3D được dựng bằng Three.js, KHÔNG phải asset URDF/SDF, mesh nguyên bản OMTS hoặc mô phỏng vật lý của Gazebo.
- Động học tay robot sử dụng xấp xỉ IK hình học hai đoạn trong không gian 3D; KHÔNG phải mô hình kinematics/động lực học UR5e, KHÔNG kiểm soát giới hạn khớp, va chạm hay tự tính quỹ đạo bằng MoveIt.
- Hộp nhận diện và tọa độ là dữ liệu tổng hợp; KHÔNG dùng NVIDIA FoundationPose, không nhận camera thật.
- Đèn spindle, cửa CNC, ê-tô, liên động và tín hiệu I/O đều là hoạt hình / state machine trên trình duyệt; **không phải PLC thực, không phải chứng nhận an toàn**, không gửi lệnh tới robot thật.
- Mọi trạng thái chạy bên trong trình duyệt, không gọi API robot và không mở cổng điều khiển robot.
- Không triển khai mã minh họa này lên dây chuyền sản xuất thật. Để chạy mô phỏng vật lý thật, xem [Intrinsic Core Getting Started](https://github.com/intrinsic-ai/intrinsic-core/blob/main/developer_resources/learn/tutorials/getting_started.md), chạy OMTS với `--operation_mode=sim`, và làm theo tài liệu thiết lập an toàn tương ứng.

## Kiến trúc nguồn

- `index.html`: DOM dashboard, hệ thống nút bấm, biểu diễn trạng thái.
- `style.css`: thiết kế responsive desktop / mobile.
- `app.js`: dựng không gian Three.js, robot hai liên kết, robot cell, CNC, giả lập camera, bảng timeline tám bước.
- `README.md`: tài liệu Việt hóa.

## Nguồn và giấy phép

Phát triển tài liệu/demo cộng đồng trên repository fork của [Intrinsic Core](https://github.com/intrinsic-ai/intrinsic-core) (Apache-2.0), tham khảo [Intrinsic OMTS](https://github.com/intrinsic-ai/intrinsic-omts) (Apache-2.0). Tác giả phần mềm gốc và nhãn hiệu thuộc các chủ sở hữu tương ứng. Đây không phải ứng dụng chính thức hay được Google/Intrinsic chứng thực. Three.js được tải từ CDN theo giấy phép MIT của thư viện.
