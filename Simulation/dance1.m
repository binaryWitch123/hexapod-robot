% =========================================================================
% MO PHONG ROBOT HEXAPOD - FIX: CHAN DINH DAT, THAN LAC LU
% =========================================================================
clc; clear; close all;

%% ========================================================================
%  PHAN 1: THIET LAP DU LIEU & CAU HINH
% ========================================================================
STATES_DATA = [
    % STATE 1: Raise Hands (Trang thai chuan - chan cham dat)
    0, 45, -75,   0, 45, -75,   0, 45, 75,   0, 45, 75,   0, 45, -75,   0, 45, -75;
    % STATE 2: Lean Left
    6.02, 42.43, -69.13,   7.61, 49.13, -79.53,   0.00, 40.89, 30.00,   0.00, 50.22, 30.00,  -6.02, 42.43, -69.13,  -7.61, 49.13, -79.53;
    % STATE 3: Lean Right
p'    -7.61, 49.13, -79.53,  -6.02, 42.43, -69.13,   0.00, 50.22, 120.00,   0.00, 40.89, 120.00,   7.61, 49.13, -79.53,   6.02, 42.43, -69.13;
%5];
%STATE_NAMES = {'1. Raise Hands', '2. Lean Left', '3. Lean Right'};

STATES_DATA = [
    % --- MOVEMENT STATES (1-7) ---
    % STATE 1: STAND
    0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75;
    
    % STATE 2: Tripod A Lift (FL, MR, RL Lift)
    0, 75, -95,   0, 45, -75,   0, 45, -75,   0, 75, -95,   0, 75, -95,   0, 45, -75;
    
    % STATE 3: Tripod A Forward
    22, 75, -95, -22, 45, -75, -22, 45, -75,  22, 75, -95,  22, 75, -95, -22, 45, -75;
    
    % STATE 4: Tripod A Land
    22, 45, -75, -22, 45, -75, -22, 45, -75,  22, 45, -75,  22, 45, -75, -22, 45, -75;
    
    % STATE 5: Tripod B Lift (FR, ML, RR Lift)
    0, 45, -75,   0, 75, -95,   0, 75, -95,   0, 45, -75,   0, 45, -75,   0, 75, -95;
    
    % STATE 6: Tripod B Forward
   -22, 45, -75,  22, 75, -95,  22, 75, -95, -22, 45, -75, -22, 45, -75,  22, 75, -95;
    
    % STATE 7: Tripod B Land
   -22, 45, -75,  22, 45, -75,  22, 45, -75, -22, 45, -75, -22, 45, -75,  22, 45, -75;

    % --- SIT STATES (8-9) ---
    % STATE 8: SIT (Low)
    0.00, 102.57, -35.85,   0.00, 102.57, -35.85,   0.00, 102.57, -35.85,   0.00, 102.57, -35.85,   0.00, 102.57, -35.85,   0.00, 102.57, -35.85;

    % STATE 9: SIT (High/Ready)
    0.00, 66.04, -33.25,    0.00, 66.04, -33.25,    0.00, 66.04, -33.25,    0.00, 66.04, -33.25,    0.00, 66.04, -33.25,    0.00, 66.04, -33.25;
];

STATE_NAMES = {
    '1. STAND', 
    '2. Tripod A Lift', 
    '3. Tripod A Forward', 
    '4. Tripod A Land', 
    '5. Tripod B Lift', 
    '6. Tripod B Forward', 
    '7. Tripod B Land',
    '8. SIT (Low)',
    '9. SIT (High)'
};

% Kich thuoc Robot
BODY_LENGTH = 150.703; BODY_WIDTH = 70.712; MID_SPAN = 120.0;   
L_COXA = 35; L_FEMUR = 75; L_TIBIA = 125;

% Tinh chieu cao mac dinh (h) dua tren trang thai 1
delta_z = L_FEMUR * sind(45) + L_TIBIA * sind(-75); 
BODY_HEIGHT = abs(delta_z); 
h = BODY_HEIGHT;

half_L = BODY_LENGTH/2; half_W = BODY_WIDTH/2; half_Mid = MID_SPAN/2;

% Dinh nghia cac diem gan chan (Mount Points) trong he quy chieu Local cua Body (Z=0)
% Luu y: O day ta de Z=0 vi ta se tinh toan moi thu tu tam Body
mount_pts_local = [
     half_L, -half_W, 0;   0, -half_Mid, 0;   -half_L, -half_W, 0;
     half_L,  half_W, 0;   0,  half_Mid, 0;   -half_L,  half_W, 0
];
mount_angles = [-45, -90, -135, 45, 90, 135];

%% ========================================================================
%  PHAN 2: TINH TOAN VI TRI CHAN "DINH MAP" (REFERENCE TIPS)
% ========================================================================
% Ta gia su o State 1, Robot dung chuan, Body o do cao h
% Day se la cac diem ma chan phai "khoa" vao do
angles_std = reshape_angles(STATES_DATA(1,:));
[ref_tips, ~] = forward_kinematics(angles_std, mount_pts_local, mount_angles, L_COXA, L_FEMUR, L_TIBIA);
% Dich chuyen ref_tips len do cao h de mo phong viec than robot dang o tren cao
% Khi do Tip thuc te se o Z ~ 0
ref_tips_world = ref_tips + [0, 0, h]; 

%% ========================================================================
%  PHAN 3: VONG LAP MO PHONG
% ========================================================================
fig_h = figure('Name', 'Hexapod Body Sway Simulation', 'Color', 'white', 'Position', [100 100 800 600]);
axis equal; grid on; view(30, 20); hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
zlim([0 300]); xlim([-300 300]); ylim([-300 300]);

% Ve san (Ground)
patch([400 -400 -400 400], [400 400 -400 -400], [0 0 0 0], [0.9 0.9 0.9], 'EdgeColor', 'none');

steps = 30; % Tang so buoc de muot hon
num_states = size(STATES_DATA, 1);

for cycle = 1:5 
    for s = 1:num_states
        if ~isvalid(fig_h), break; end
        
        current_state_raw = STATES_DATA(s, :);
        next_state_idx = mod(s, num_states) + 1;
        next_state_raw = STATES_DATA(next_state_idx, :);
        
        for t = linspace(0, 1, steps)
            if ~isvalid(fig_h), break; end
            
            % 1. Noi suy goc (Interpolation)
            interp_raw = (1-t)*current_state_raw + t*next_state_raw;
            CURRENT_ANGLES = reshape_angles(interp_raw);
            
            % 2. Tinh toan hinh dang Robot hien tai (trong he quy chieu Body)
            % Tips_local: toa do mui chan neu Body o (0,0,0)
            [tips_local, joints_local] = forward_kinematics(CURRENT_ANGLES, mount_pts_local, mount_angles, L_COXA, L_FEMUR, L_TIBIA);
            
            % 3. THUAT TOAN KHOA CHAN (BODY TRANSFORMATION)
            % Tim ma tran R (Xoay) va T (Dich) de: (R * tips_local + T) ~ ref_tips_world
            [R, T] = align_points(tips_local, ref_tips_world);
            
            % 4. Ve Robot
            delete(findobj(gca, 'Tag', 'robot_part')); % Xoa khung hinh cu
            
            % Ve cac diem "Dinh map" de kiem chung (Diem mau do duoi dat)
            plot3(ref_tips_world(:,1), ref_tips_world(:,2), ref_tips_world(:,3), 'r+', 'MarkerSize', 5, 'Tag', 'robot_part');
            
            draw_transformed_robot(joints_local, mount_pts_local, R, T);
            
            % Ve mui ten huong dau (Head Vector) cung phai xoay theo than
            head_start = (R * [0; 0; 0] + T)'; 
            head_vec   = (R * [200; 0; 0])';
            quiver3(head_start(1), head_start(2), head_start(3), ...
                    head_vec(1), head_vec(2), head_vec(3), ...
                    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off', 'Tag', 'robot_part');
            
            title({['CYCLE: ' num2str(cycle) ' | STATE: ' STATE_NAMES{s}], ...
                   'Tips Locked at Z=0, Body Sways'}, 'FontSize', 14);
            drawnow;
        end
    end
    if ~isvalid(fig_h), break; end
end

%% ========================================================================
%  CAC HAM HO TRO (HELPER FUNCTIONS)
% ========================================================================

% Ham sap xep mang du lieu thanh ma tran 6x3
function angles_mat = reshape_angles(raw_data)
    FL = raw_data(1:3); FR = raw_data(4:6); ML = raw_data(7:9);
    MR = raw_data(10:12); RL = raw_data(13:15); RR = raw_data(16:18);
    angles_mat = [FR; MR; RR; FL; ML; RL];
end

% Ham tinh dong hoc thuan (Tra ve toa do Tip va cac khop trong he quy chieu Body)
function [tips, all_joints] = forward_kinematics(angles_mat, mount_pts, mount_angles, L1, L2, L3)
    tips = zeros(6, 3);
    all_joints = zeros(6, 4, 3); % 6 chan, 4 diem (J1..J4), 3 toa do xyz
    
    for i = 1:6
        th1 = angles_mat(i, 1); th2 = angles_mat(i, 2); th3 = angles_mat(i, 3);
        alpha = mount_angles(i);
        total_rad = deg2rad(alpha + th1);
        
        J1 = mount_pts(i,:);
        J2 = J1 + [L1*cos(total_rad), L1*sin(total_rad), L1*sind(th1)];
        
        r_femur = L2*cosd(th2); z_femur = L2*sind(th2);
        J3 = J2 + [r_femur*cos(total_rad), r_femur*sin(total_rad), z_femur];
        
        r_tibia = L3*cosd(th3); z_tibia = L3*sind(th3);
        J4 = J3 + [r_tibia*cos(total_rad), r_tibia*sin(total_rad), z_tibia];
        
        tips(i, :) = J4;
        all_joints(i,:,:) = [J1; J2; J3; J4];
    end
end

% Thuat toan Kabsch (Tim ma tran xoay va dich chuyen toi uu de khop 2 tap hop diem)
function [R, T] = align_points(P_local, P_target)
    % 1. Tim trong tam (Centroid)
    centroid_local = mean(P_local);
    centroid_target = mean(P_target);
    
    % 2. Dua ve tam
    P_local_centered = P_local - centroid_local;
    P_target_centered = P_target - centroid_target;
    
    % 3. Tinh ma tran hiep phuong sai
    H = P_local_centered' * P_target_centered;
    
    % 4. Phan ra SVD (Singular Value Decomposition)
    [U, ~, V] = svd(H);
    
    % 5. Tinh ma tran Xoay (Rotation)
    R = V * U';
    
    % Kiem tra phan xa (Reflection case)
    if det(R) < 0
        V(:,3) = -V(:,3);
        R = V * U';
    end
    
    % 6. Tinh vector Dich chuyen (Translation)
    T = centroid_target' - R * centroid_local';
end

% Ham ve robot da duoc bien doi (Transformed)
function draw_transformed_robot(all_joints, mount_pts, R, T)
    % Bien doi va ve Than
    num_legs = size(mount_pts, 1);
    mount_trans = (R * mount_pts' + T)';
    
    hex_idx = [1, 2, 3, 6, 5, 4, 1];
    fill3(mount_trans(hex_idx,1), mount_trans(hex_idx,2), mount_trans(hex_idx,3), ...
         [0.3 0.3 0.3], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'Tag', 'robot_part');
     
    % Bien doi va ve Chan
    cols = lines(6);
    for i=1:num_legs
        % Lay toa do chan i (4 diem x 3 toa do)
        limb_pts = squeeze(all_joints(i,:,:)); % [4x3]
        
        % Ap dung bien doi R, T cho tung khop cua chan
        limb_trans = (R * limb_pts' + T)';
        
        X = limb_trans(:,1); Y = limb_trans(:,2); Z = limb_trans(:,3);
        
        plot3(X, Y, Z, 'LineWidth', 4, 'Color', cols(i,:), 'Tag', 'robot_part');
        plot3(X, Y, Z, 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'w', 'Color', 'k', 'Tag', 'robot_part');
    end
end