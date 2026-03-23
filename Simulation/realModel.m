% =========================================================================
% HEXAPOD PHYSICS - TRUE INVERSE KINEMATICS (NO SLIPPAGE)
% Nguyên lý vật lý:
% 1. Chân trụ (Stance) được KHÓA chặt toạ độ trong World Frame.
% 2. Thân robot di chuyển và xoay theo đường dẫn.
% 3. Tính toán hình dáng chân dựa trên khoảng cách (Chân World - Thân World).
% =========================================================================
clc; clear; close all;

%% 1. CẤU HÌNH ROBOT & MAP
BODY_L = 150.7; BODY_W = 70.7; MID_SPAN = 120.0;
L1 = 35; L2 = 75; L3 = 125;
STRIDE = 120;        % Sải chân lớn hơn để thấy rõ bước
LIFT_H = 40;        % Độ cao nhấc chân
SPEED_FWD = 2.5;    % Tốc độ di chuyển của thân (mm/frame)

% Vị trí gắn chân trên thân (Local Frame)
Mount_Local = [
     BODY_L/2, -BODY_W/2, 0;   % 1: Trước Phải
     0,        -MID_SPAN/2, 0; % 2: Giữa Phải
    -BODY_L/2, -BODY_W/2, 0;   % 3: Sau Phải
     BODY_L/2,  BODY_W/2, 0;   % 4: Trước Trái
     0,         MID_SPAN/2, 0; % 5: Giữa Trái
    -BODY_L/2,  BODY_W/2, 0    % 6: Sau Trái
];
Mount_Angles = [-45, -90, -135, 45, 90, 135];
R_NEUTRAL = L1 + L2*cosd(45) + L3*cosd(-75); % Bán kính nghỉ
BODY_H = abs(L1*sind(0) + L2*sind(45) + L3*sind(-75));

%% 2. KHỞI TẠO ĐỒ HỌA
figure('Name', 'Physically Accurate Hexapod', 'Color', 'white');
hold on; axis equal; grid on; view(30, 45);
xlabel('X'); ylabel('Y'); zlabel('Z');

% Map giới hạn
MAP_LIM = [-500, 3000, -1000, 1000];
xlim(MAP_LIM(1:2)); ylim(MAP_LIM(3:4)); zlim([0 400]);

% Vẽ đường dẫn cong
path_x = MAP_LIM(1):10:MAP_LIM(2);
path_y = 600 * sin(path_x / 600);
plot3(path_x, path_y, zeros(size(path_x)), '--k', 'LineWidth', 1);

% Khởi tạo Handle vẽ
legs_h = gobjects(6,1);
tips_h = gobjects(6,1);
for i=1:6
    legs_h(i) = plot3(0,0,0, 'LineWidth', 2);
    tips_h(i) = plot3(0,0,0, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
end
body_h = patch('FaceColor', [0.2 0.2 0.2], 'FaceAlpha', 0.8);

%% 3. TRẠNG THÁI KHỞI TẠO (INITIAL STATE)
global_pos = [0, 0, BODY_H]; 
global_yaw = atan(cos(0)); % Góc hướng ban đầu theo tiếp tuyến

% --- QUẢN LÝ CHÂN (PHYSICS STATE) ---
% World_Foot_Pos: Lưu toạ độ thực tế của 6 đầu mũi chân trong không gian
World_Foot_Pos = zeros(6, 3);
Target_Foot_Pos = zeros(6, 3); % Điểm đến dự kiến cho chân đang bước
Liftoff_Foot_Pos = zeros(6, 3); % Điểm bắt đầu nhấc chân

% Khởi tạo chân ở vị trí nghỉ (Neutral)
for i=1:6
    ang = deg2rad(Mount_Angles(i)) + global_yaw;
    mount_gx = global_pos(1) + Mount_Local(i,1)*cos(global_yaw) - Mount_Local(i,2)*sin(global_yaw);
    mount_gy = global_pos(2) + Mount_Local(i,1)*sin(global_yaw) + Mount_Local(i,2)*cos(global_yaw);
    
    World_Foot_Pos(i, :) = [mount_gx + R_NEUTRAL*cos(ang), mount_gy + R_NEUTRAL*sin(ang), 0];
end

% Gait Parameters
gait_progress = 0; % 0 -> 1
tripod_case = 1;   % 1: Nhóm A bước, 2: Nhóm B bước
GroupA = [1, 3, 5]; % Chân 1, 3, 5
GroupB = [2, 4, 6]; % Chân 2, 4, 6

%% 4. VÒNG LẶP CHÍNH
while ishandle(body_h)
    
    % --- A. CẬP NHẬT VỊ TRÍ THÂN (BODY MOVEMENT) ---
    % Thân di chuyển độc lập theo đường dẫn
    current_x = global_pos(1);
    
    % Tính Heading (Yaw) theo đạo hàm đường cong: y' = cos(x/600)
    tangent = cos(current_x / 600); 
    target_yaw = atan(tangent);
    
    % Làm mượt góc quay (giả lập quán tính)
    global_yaw = global_yaw * 0.8 + target_yaw * 0.2;
    
    % Di chuyển thân tới trước theo hướng Yaw
    dx = SPEED_FWD * cos(global_yaw);
    dy = SPEED_FWD * sin(global_yaw);
    global_pos(1) = global_pos(1) + dx;
    global_pos(2) = global_pos(2) + dy;
    
    % Camera follow
    xlim([global_pos(1)-600, global_pos(1)+900]);
    ylim([global_pos(2)-750, global_pos(2)+750]);

    % --- B. CẬP NHẬT TRẠNG THÁI CHÂN (GAIT LOGIC) ---
    gait_progress = gait_progress + 0.01; % Tốc độ chu kỳ bước
    
    % Chuyển pha (Switch Phase)
    if gait_progress >= 1
        gait_progress = 0;
        tripod_case = 3 - tripod_case; % Đổi nhóm: 1->2 hoặc 2->1
        
        % KHI CHUYỂN PHA: Tính điểm đáp (Target) cho nhóm chân chuẩn bị bước
        % Dự đoán vị trí thân xe tại cuối chu kỳ bước tiếp theo
        future_body_x = global_pos(1) + (SPEED_FWD * (1/0.04)) * cos(global_yaw);
        future_body_y = global_pos(2) + (SPEED_FWD * (1/0.04)) * sin(global_yaw);
        
        moving_idx = [];
        if tripod_case == 1, moving_idx = GroupA; else, moving_idx = GroupB; end
        
        for idx = moving_idx
            % Lưu vị trí hiện tại để làm điểm bắt đầu (Lift off)
            Liftoff_Foot_Pos(idx, :) = World_Foot_Pos(idx, :);
            
            % Tính điểm đáp lý tưởng (Ideal Landing)
            % = Future_Body_Mount_Point + Neutral_Vector (xoay theo Yaw)
            mx_local = Mount_Local(idx, 1); my_local = Mount_Local(idx, 2);
            
            % Toạ độ Mount Point trong tương lai
            mount_future_x = future_body_x + mx_local*cos(global_yaw) - my_local*sin(global_yaw);
            mount_future_y = future_body_y + mx_local*sin(global_yaw) + my_local*cos(global_yaw);
            
            % Hướng chân neutral
            foot_ang = deg2rad(Mount_Angles(idx)) + global_yaw;
            
            Target_Foot_Pos(idx, :) = [
                mount_future_x + R_NEUTRAL * cos(foot_ang), ...
                mount_future_y + R_NEUTRAL * sin(foot_ang), ...
                0];
        end
    end
    
    % --- C. TÍNH TOÁN VỊ TRÍ TỪNG CHÂN (INVERSE KINEMATICS) ---
    if tripod_case == 1
        swing_legs = GroupA; stance_legs = GroupB;
    else
        swing_legs = GroupB; stance_legs = GroupA;
    end
    
    % 1. XỬ LÝ CHÂN TRỤ (STANCE) - QUAN TRỌNG NHẤT
    % Không làm gì cả! Toạ độ World_Foot_Pos giữ nguyên.
    % Điều này đảm bảo chân "dính" chặt xuống sàn, không bị trượt khi thân xoay.
    
    % 2. XỬ LÝ CHÂN BƯỚC (SWING)
    for idx = swing_legs
        % Nội suy tuyến tính (Linear Interpolation) cho X, Y
        World_Foot_Pos(idx, 1) = Liftoff_Foot_Pos(idx, 1) + (Target_Foot_Pos(idx, 1) - Liftoff_Foot_Pos(idx, 1)) * gait_progress;
        World_Foot_Pos(idx, 2) = Liftoff_Foot_Pos(idx, 2) + (Target_Foot_Pos(idx, 2) - Liftoff_Foot_Pos(idx, 2)) * gait_progress;
        % Parabol cho Z (nhấc chân)
        World_Foot_Pos(idx, 3) = sin(gait_progress * pi) * LIFT_H;
    end
    
    % --- D. VẼ ROBOT (VISUALIZATION) ---
    % Vẽ Thân
    body_pts_x = zeros(1,7); body_pts_y = zeros(1,7);
    idx_list = [1 2 3 6 5 4 1];
    for k=1:7
        idx = idx_list(k);
        lx = Mount_Local(idx, 1); ly = Mount_Local(idx, 2);
        gx = lx * cos(global_yaw) - ly * sin(global_yaw);
        gy = lx * sin(global_yaw) + ly * cos(global_yaw);
        body_pts_x(k) = global_pos(1) + gx;
        body_pts_y(k) = global_pos(2) + gy;
    end
    set(body_h, 'XData', body_pts_x, 'YData', body_pts_y, 'ZData', ones(1,7)*global_pos(3));
    
    % Vẽ Chân
    for i=1:6
        % Toạ độ khớp hông (J1) trong World Frame
        mx = Mount_Local(i,1); my = Mount_Local(i,2);
        J1_x = global_pos(1) + mx*cos(global_yaw) - my*sin(global_yaw);
        J1_y = global_pos(2) + mx*sin(global_yaw) + my*cos(global_yaw);
        J1_z = global_pos(3);
        
        % Toạ độ mũi chân (J4) là World_Foot_Pos (Đã tính ở trên)
        J4 = World_Foot_Pos(i, :);
        
        % Giả lập IK đơn giản để vẽ khớp gối cho đẹp (không ảnh hưởng physics)
        vec_x = J4(1) - J1_x; vec_y = J4(2) - J1_y; dist = sqrt(vec_x^2 + vec_y^2);
        angle_leg = atan2(vec_y, vec_x);
        
        J2_x = J1_x + L1 * cos(angle_leg);
        J2_y = J1_y + L1 * sin(angle_leg);
        J2_z = J1_z;
        
        mid_x = (J2_x + J4(1))/2; mid_y = (J2_y + J4(2))/2;
        J3_x = mid_x; J3_y = mid_y; J3_z = J1_z + L2*0.5; % Gối nhô lên
        
        % Update đồ thị
        if ismember(i, stance_legs), col = 'r'; else, col = 'b'; end
        set(legs_h(i), 'XData', [J1_x J2_x J3_x J4(1)], ...
                       'YData', [J1_y J2_y J3_y J4(2)], ...
                       'ZData', [J1_z J2_z J3_z J4(3)], 'Color', col);
        set(tips_h(i), 'XData', J4(1), 'YData', J4(2), 'ZData', J4(3), 'MarkerFaceColor', col);
    end
    
    drawnow limitrate;
end