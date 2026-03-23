% =========================================================================
% MO PHONG ROBOT HEXAPOD - FIX LOI CUA SO VA LABEL
% Input: Chuoi trang thai (State Sequence) tu User
% =========================================================================
clc; clear; close all;

%% ========================================================================
%  PHAN 1: THIET LAP DU LIEU CHUYEN DONG (DATA INPUT)
% ========================================================================
STATES_DATA = [
    % STATE 1: STAND
    0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75;
    % STATE 2: LIFT
    0, 75, -95,   0, 45, -75,   0, 45, -75,   0, 75, -95,   0, 75, -95,   0, 45, -75;
    % STATE 3: FORWARD
    22, 75, -95, -22, 45, -75, -22, 45, -75,  22, 75, -95,  22, 75, -95, -22, 45, -75;
    % STATE 4: LAND
    22, 45, -75, -22, 45, -75, -22, 45, -75,  22, 45, -75,  22, 45, -75, -22, 45, -75;
];
STATE_NAMES = {'1. STAND', '2. LIFT', '3. FORWARD', '4. LAND'};

%% ========================================================================
%  PHAN 2: THONG SO ROBOT
% ========================================================================
BODY_LENGTH = 150.703; BODY_WIDTH = 70.712; MID_SPAN = 120.0;   
L_COXA = 35; L_FEMUR = 75; L_TIBIA = 125;

delta_z = L_FEMUR * sind(45) + L_TIBIA * sind(-75);
BODY_HEIGHT = abs(delta_z); 
h = BODY_HEIGHT;
half_L = BODY_LENGTH/2; half_W = BODY_WIDTH/2; half_Mid = MID_SPAN/2;

mount_pts = [
     half_L, -half_W, h;   0, -half_Mid, h;   -half_L, -half_W, h;
     half_L,  half_W, h;   0,  half_Mid, h;   -half_L,  half_W, h
];
mount_angles = [-45, -90, -135, 45, 90, 135];

%% ========================================================================
%  PHAN 3: VONG LAP MO PHONG (ANIMATION LOOP)
% ========================================================================
% Luu handle cua cua so vao bien fig_h de kiem tra
fig_h = figure('Name', 'Hexapod Motion Simulation', 'Color', 'white', 'Position', [100 100 800 600]);
axis equal; grid on; view(30, 30); hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
zlim([0 300]); xlim([-300 300]); ylim([-300 300]);

% Ve san va mui ten (Nhung thu nay ve 1 lan, khong xoa)
patch([400 -400 -400 400], [400 400 -400 -400], [0 0 0 0], [0.95 0.95 0.95], 'EdgeColor', 'none');
quiver3(-100, 0, h+50, 200, 0, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
text(120, 0, h+50, 'HEAD', 'Color', 'r', 'FontWeight', 'bold');

steps = 20; 
num_states = size(STATES_DATA, 1);

for cycle = 1:3 
    for s = 1:num_states
        
        % KIEM TRA AN TOAN: Neu nguoi dung tat cua so thi dung chuong trinh
        if ~isvalid(fig_h), break; end
        
        current_state_raw = STATES_DATA(s, :);
        if s < num_states
            next_state_raw = STATES_DATA(s+1, :);
        else
            next_state_raw = STATES_DATA(1, :);
        end
        
        for t = linspace(0, 1, steps)
            % KIEM TRA AN TOAN TRONG VONG LAP CON
            if ~isvalid(fig_h), break; end
            
            interp_raw = (1-t)*current_state_raw + t*next_state_raw;
            
            FL = interp_raw(1:3); FR = interp_raw(4:6); ML = interp_raw(7:9);
            MR = interp_raw(10:12); RL = interp_raw(13:15); RR = interp_raw(16:18);
            
            CURRENT_ANGLES = [FR; MR; RR; FL; ML; RL];
            
            % --- QUAN TRONG: CHI XOA DOI TUONG ROBOT ---
            delete(findobj(gca, 'Tag', 'robot_part')); 
            
            draw_robot(CURRENT_ANGLES, mount_pts, mount_angles, L_COXA, L_FEMUR, L_TIBIA);
            
            title({['CYCLE: ' num2str(cycle) ' | STATE: ' STATE_NAMES{s}], ...
                   'Tripod Gait Simulation'}, 'FontSize', 14);
            drawnow;
        end
    end
    if ~isvalid(fig_h), break; end
end

%% ========================================================================
%  CAC HAM HO TRO (HELPER FUNCTIONS)
% ========================================================================

function draw_robot(angles_mat, mount_pts, mount_angles, L1, L2, L3)
    joints_pos = zeros(6, 4, 3);
    
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
        
        joints_pos(i,:,:) = [J1; J2; J3; J4];
    end
    
    % Ve than (Them Tag 'robot_part')
    hex_idx = [1, 2, 3, 6, 5, 4, 1];
    fill3(mount_pts(hex_idx,1), mount_pts(hex_idx,2), mount_pts(hex_idx,3), ...
         [0.3 0.3 0.3], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'Tag', 'robot_part');
     
    % Ve chan (Them Tag 'robot_part')
    cols = lines(6);
    for i=1:6
        X = squeeze(joints_pos(i,:,1));
        Y = squeeze(joints_pos(i,:,2));
        Z = squeeze(joints_pos(i,:,3));
        % Tag giup ta xoa dung chan chu khong xoa truc toa do
        plot3(X, Y, Z, 'LineWidth', 4, 'Color', cols(i,:), 'Tag', 'robot_part');
        plot3(X, Y, Z, 'o', 'MarkerSize', 4, 'MarkerFaceColor', 'w', 'Color', 'k', 'Tag', 'robot_part');
    end
end