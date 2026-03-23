% =========================================================================
% HEXAPOD PHYSICS - CURVED PATH TRACKING (SINE WAVE)
% Nguyên lý:
% 1. Tính toán bước đi trong hệ Local (thẳng).
% 2. Xoay toàn bộ hệ Local theo góc cong của đường dẫn (Heading).
% 3. Cộng dồn vào tọa độ Global.
% =========================================================================
clc; clear; close all;

%% 1. THÔNG SỐ CƠ KHÍ & BƯỚC ĐI
BODY_L = 150.7; BODY_W = 70.7; MID_SPAN = 120.0;
L1 = 35; L2 = 75; L3 = 125;

STRIDE = 60;     % Sải chân (mm)
LIFT_H = 30;     % Độ cao nhấc chân (mm)
SPEED  = 0.1;   % Tốc độ mô phỏng

% Điểm gắn chân (Local Frame)
Mount_Local = [
     BODY_L/2, -BODY_W/2, 0;   % 1
     0,        -MID_SPAN/2, 0; % 2
    -BODY_L/2, -BODY_W/2, 0;   % 3
     BODY_L/2,  BODY_W/2, 0;   % 4
     0,         MID_SPAN/2, 0; % 5
    -BODY_L/2,  BODY_W/2, 0    % 6
];
Mount_Angles = [-45, -90, -135, 45, 90, 135];

% Chiều cao thân xe
Neutral_Z = L1*sind(0) + L2*sind(45) + L3*sind(-75);
BODY_H = abs(Neutral_Z); 

%% 2. KHỞI TẠO MÔI TRƯỜNG RỘNG LỚN
figure('Name', 'Curved Path Navigation', 'Color', 'white');
hold on; axis equal; grid on; view(30, 45);
xlabel('X (World)'); ylabel('Y (World)'); zlabel('Z (World)');

% --- MAP CONFIGURATION ---
MAP_X_MIN = -500; MAP_X_MAX = 3500;
MAP_Y_MIN = -1500; MAP_Y_MAX = 1500;
xlim([MAP_X_MIN MAP_X_MAX]); ylim([MAP_Y_MIN MAP_Y_MAX]); zlim([0 400]);

% Vẽ sàn lưới rộng
patch([MAP_X_MIN MAP_X_MAX MAP_X_MAX MAP_X_MIN], ...
      [MAP_Y_MAX MAP_Y_MAX MAP_Y_MIN MAP_Y_MIN], [0 0 0 0], ...
      [0.95 0.95 0.95], 'EdgeColor', 'none');
for x = MAP_X_MIN:200:MAP_X_MAX
    plot3([x x], [MAP_Y_MIN MAP_Y_MAX], [0 0], 'Color', [0.85 0.85 0.85]);
end
for y = MAP_Y_MIN:200:MAP_Y_MAX
    plot3([MAP_X_MIN MAP_X_MAX], [y y], [0 0], 'Color', [0.85 0.85 0.85]);
end

% --- VẼ ĐƯỜNG DẪN CONG (VISUAL PATH) ---
% Đường dẫn: Y = 500 * sin(X / 500)
path_x = MAP_X_MIN:50:MAP_X_MAX;
path_y = 600 * sin(path_x / 600); 
plot3(path_x, path_y, ones(size(path_x)), '--b', 'LineWidth', 1);

% Tạo Handle đồ họa
legs_h = gobjects(6,1);
tips_h = gobjects(6,1);
for i=1:6
    legs_h(i) = plot3(0,0,0, 'LineWidth', 2);
    tips_h(i) = plot3(0,0,0, 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
end
body_h = patch('FaceColor', [0.2 0.2 0.2], 'FaceAlpha', 0.9);
title_h = title('Initializing...');

%% 3. VÒNG LẶP CHÍNH
global_pos = [0, 0, BODY_H]; % [X, Y, Z] trong không gian
global_yaw = 0;              % Góc hướng đầu của robot (rad)
t = 0;

while ishandle(body_h)
    
    % --- GAIT PHASE CALCULATION ---
    cycle = mod(t, 2*pi);
    if cycle < pi
        percent = cycle / pi;
        mode_tri1 = 'STANCE'; mode_tri2 = 'SWING';
    else
        percent = (cycle - pi) / pi;
        mode_tri1 = 'SWING'; mode_tri2 = 'STANCE';
    end
    
    % --- PHYSICS: TÍNH ĐỘ TRƯỢT THÂN XE ---
    % Giả sử chân trụ bám đất hoàn hảo -> Thân tịnh tiến
    % Tính độ dời chân Local
    curr_stance_local = (STRIDE/2) * cos(percent * pi);
    prev_percent = max(0, percent - (SPEED/pi));
    prev_stance_local = (STRIDE/2) * cos(prev_percent * pi);
    
    % Độ dịch chuyển thẳng (Scalar distance)
    move_dist = abs(curr_stance_local - prev_stance_local);
    
    % --- NAVIGATION: CẬP NHẬT VỊ TRÍ & GÓC QUAY ---
    % 1. Tính góc hướng (Heading) dựa trên đạo hàm đường cong
    % Y = A*sin(B*x) -> Y' = A*B*cos(B*x) -> Angle = atan(Y')
    % Ở đây x là global_pos(1)
    current_x = global_pos(1);
    tangent_slope = (600/600) * cos(current_x / 600); % Đạo hàm của 600*sin(x/600)
    target_yaw = atan(tangent_slope);
    
    % Làm mượt góc quay (để robot không giật cục)
    global_yaw = target_yaw; 
    
    % 2. Cập nhật tọa độ Global (X, Y) theo hướng quay
    dx = move_dist * cos(global_yaw);
    dy = move_dist * sin(global_yaw);
    
    global_pos(1) = global_pos(1) + dx;
    global_pos(2) = global_pos(2) + dy;
    
    % Camera Tracking
    xlim([global_pos(1)-500, global_pos(1)+1000]);
    ylim([global_pos(2)-800, global_pos(2)+800]);

    % --- KINEMATICS UPDATE ---
    for i = 1:6
        % Xác định mode chân
        if ismember(i, [1 3 5]), mode = mode_tri1; else, mode = mode_tri2; end
        
        % 1. TÍNH LOCAL (Như đi thẳng)
        ang_rad = deg2rad(Mount_Angles(i));
        R_neutral = L1 + L2*cosd(45) + L3*cosd(-75);
        
        if strcmp(mode, 'STANCE')
            local_offset_x = (STRIDE/2) * cos(percent * pi);
            local_z = -BODY_H;
            color = 'r';
        else
            local_offset_x = -(STRIDE/2) * cos(percent * pi);
            local_z = -BODY_H + LIFT_H * sin(percent * pi);
            color = 'b';
        end
        
        % Tọa độ mũi chân trong hệ Local (Thẳng)
        tip_local_x = (R_neutral * cos(ang_rad)) + local_offset_x;
        tip_local_y = (R_neutral * sin(ang_rad));
        tip_local_z = local_z;
        
        % 2. TRANSFORMATION: LOCAL -> GLOBAL (Có xoay)
        % Ma trận quay 2D quanh trục Z
        % [ x' ]   [ cos -sin ] [ x ]
        % [ y' ] = [ sin  cos ] [ y ]
        
        % Vector từ tâm thân đến Mũi chân (Local)
        vec_x = tip_local_x; % Lưu ý: tip_local tính từ tâm thân
        vec_y = tip_local_y;
        
        % Xoay vector này theo Global Yaw
        rot_x = vec_x * cos(global_yaw) - vec_y * sin(global_yaw);
        rot_y = vec_x * sin(global_yaw) + vec_y * cos(global_yaw);
        
        % Cộng vào tọa độ tâm thân Global
        J4_Global = global_pos + [rot_x, rot_y, tip_local_z];
        
        % Tính lại các khớp J1, J2, J3 để vẽ
        % J1 (Mount Point) Global
        mx = Mount_Local(i,1); my = Mount_Local(i,2);
        rmx = mx * cos(global_yaw) - my * sin(global_yaw);
        rmy = mx * sin(global_yaw) + my * cos(global_yaw);
        J1_Global = global_pos + [rmx, rmy, 0];
        
        % IK đơn giản để vẽ khớp gối
        V = J4_Global - J1_Global;
        angle_J1 = atan2(V(2), V(1));
        J2_Global = J1_Global + [L1*cos(angle_J1), L1*sin(angle_J1), 0];
        mid = (J2_Global + J4_Global)/2;
        J3_Global = mid + [0, 0, L2*0.8]; 
        
        % Vẽ
        line_x = [J1_Global(1) J2_Global(1) J3_Global(1) J4_Global(1)];
        line_y = [J1_Global(2) J2_Global(2) J3_Global(2) J4_Global(2)];
        line_z = [J1_Global(3) J2_Global(3) J3_Global(3) J4_Global(3)];
        
        set(legs_h(i), 'XData', line_x, 'YData', line_y, 'ZData', line_z, 'Color', color);
        set(tips_h(i), 'XData', J4_Global(1), 'YData', J4_Global(2), 'ZData', J4_Global(3), ...
            'MarkerFaceColor', color, 'Color', color);
    end
    
    % Vẽ Thân (Đã xoay)
    body_pts_x = zeros(1,7); body_pts_y = zeros(1,7);
    idx_list = [1 2 3 6 5 4 1];
    for k=1:7
        idx = idx_list(k);
        lx = Mount_Local(idx, 1); ly = Mount_Local(idx, 2);
        % Xoay điểm thân
        gx = lx * cos(global_yaw) - ly * sin(global_yaw);
        gy = lx * sin(global_yaw) + ly * cos(global_yaw);
        body_pts_x(k) = global_pos(1) + gx;
        body_pts_y(k) = global_pos(2) + gy;
    end
    set(body_h, 'XData', body_pts_x, 'YData', body_pts_y, 'ZData', ones(1,7)*BODY_H);
    
    % Update Info
    title_str = sprintf('POS: [%.0f, %.0f] | YAW: %.1f deg', ...
        global_pos(1), global_pos(2), rad2deg(global_yaw));
    set(title_h, 'String', title_str);
    
    t = t + SPEED;
    drawnow;
end