% =========================================================================
% MO PHONG ROBOT HEXAPOD - PHIEN BAN VAT LY (PHYSICS UPDATE)
% Fix: Chong lun (Z) va chong truot (Odometry/Foot Locking)
% =========================================================================
clc; clear; close all;

%% ========================================================================
%  PHAN 1: DU LIEU VA THIET LAP BAN DAU
% ========================================================================
% (Giu nguyen du lieu cua ban)
STATES_DATA = [
    % STATE 1: STAND
    0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75,   0, 45, -75;
    % STATE 2: Tripod A Lift 
    0, 75, -95,   0, 45, -75,   0, 45, -75,   0, 75, -95,   0, 75, -95,   0, 45, -75;
    % STATE 3: Tripod A Forward
    22, 75, -95, -22, 45, -75, -22, 45, -75,  22, 75, -95,  22, 75, -95, -22, 45, -75;
    % STATE 4: Tripod A Land
    22, 45, -75, -22, 45, -75, -22, 45, -75,  22, 45, -75,  22, 45, -75, -22, 45, -75;
    % STATE 5: Tripod B Lift
    0, 45, -75,   0, 75, -95,   0, 75, -95,   0, 45, -75,   0, 45, -75,   0, 75, -95;
    % STATE 6: Tripod B Forward
   -22, 45, -75,  22, 75, -95,  22, 75, -95, -22, 45, -75, -22, 45, -75,  22, 75, -95;
    % STATE 7: Tripod B Land
   -22, 45, -75,  22, 45, -75,  22, 45, -75, -22, 45, -75, -22, 45, -75,  22, 45, -75;
   
];

% State Names for display
STATE_NAMES = {'Stand', 'Tri A Lift', 'Tri A Fwd', 'Tri A Land', ...
               'Tri B Lift', 'Tri B Fwd', 'Tri B Land'};

BODY_LENGTH = 150.703; BODY_WIDTH = 70.712; MID_SPAN = 120.0;   
L_COXA = 35; L_FEMUR = 75; L_TIBIA = 125;
delta_z = L_FEMUR * sind(45) + L_TIBIA * sind(-75);
DEFAULT_HEIGHT = abs(delta_z); 

% Mounting Points (Local Body Frame)
half_L = BODY_LENGTH/2; half_W = BODY_WIDTH/2; half_Mid = MID_SPAN/2;
% Luu y: Z o day = 0 trong Local Frame, ta se cong Global Z sau
mount_pts_local = [
     half_L, -half_W, 0;   0, -half_Mid, 0;   -half_L, -half_W, 0;
     half_L,  half_W, 0;   0,  half_Mid, 0;   -half_L,  half_W, 0
];
mount_angles = [-45, -90, -135, 45, 90, 135];

%% ========================================================================
%  PHAN 2: KHOI TAO DO HOA
% ========================================================================
fig_h = figure('Name', 'Hexapod Physics Simulation', 'Color', 'white', 'Position', [100 100 900 700]);
axis equal; grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(30, 30);

% Ve san (Ground) co dinh o Z=0
ground_size = 2000;
patch([ground_size -ground_size -ground_size ground_size], ...
      [ground_size ground_size -ground_size -ground_size], ...
      [0 0 0 0], [0.9 0.95 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
      
% Bien theo doi vi tri toan cuc cua Body (Global Position)
% [X, Y, Z]
body_global_pos = [0, 0, DEFAULT_HEIGHT]; 

% Luu vi tri chan toan cuc cua frame truoc (de tinh do truot)
prev_feet_global = []; 

%% ========================================================================
%  PHAN 3: VONG LAP MO PHONG
% ========================================================================
steps = 15; 
num_states = size(STATES_DATA, 1);

for cycle = 1:2 % Chay 2 vong lap de kiem tra di chuyen xa
    for s = 1:num_states
        if ~isvalid(fig_h), break; end
        
        current_state_raw = STATES_DATA(s, :);
        if s < num_states
            next_state_raw = STATES_DATA(s+1, :);
        else
            next_state_raw = STATES_DATA(1, :);
        end
        
        for t = linspace(0, 1, steps)
            if ~isvalid(fig_h), break; end
            
            % 1. Noi suy goc khop
            interp_raw = (1-t)*current_state_raw + t*next_state_raw;
            FL = interp_raw(1:3); FR = interp_raw(4:6); ML = interp_raw(7:9);
            MR = interp_raw(10:12); RL = interp_raw(13:15); RR = interp_raw(16:18);
            CURRENT_ANGLES = [FR; MR; RR; FL; ML; RL];
            
            % 2. Tinh Forward Kinematics (TOA DO CUC BO - CHUA CO BODY POS)
            [feet_local] = calc_fk_all_legs(CURRENT_ANGLES, mount_pts_local, mount_angles, L_COXA, L_FEMUR, L_TIBIA);
            
            % 3. XU LY VAT LY (PHYSICS ENGINE)
            % --- Buoc A: Chong Lun (Gravity Fix) ---
            % Tim chan thap nhat trong he toa do cuc bo
            min_z_local = min(feet_local(:, 3));
            % De chan thap nhat cham dat (Z=0), Body Z phai cach dat mot doan dung bang am cua min_z_local
            body_global_pos(3) = -min_z_local;
            
            % --- Buoc B: Chong Truot (Anchor Fix) ---
            % Tinh vi tri chan tam thoi trong Global neu Body khong di chuyen XY
            temp_feet_global = feet_local + body_global_pos;
            
            if ~isempty(prev_feet_global)
                % Tim cac chan dang "bam dat" (Stance phase)
                % La nhung chan co Z gan bang 0 (Z < 1mm)
                stance_indices = find(temp_feet_global(:, 3) < 1.0);
                
                % Neu co it nhat 1 chan cham dat
                if ~isempty(stance_indices)
                    % Tinh trung binh vi tri XY cua cac chan bam dat o frame truoc
                    prev_center = mean(prev_feet_global(stance_indices, 1:2), 1);
                    % Tinh trung binh vi tri XY cua cac chan do o hien tai (neu body dung yen)
                    curr_center = mean(temp_feet_global(stance_indices, 1:2), 1);
                    
                    % Do lech chinh la doan duong Body phai di chuyen de bu lai su truot chan
                    shift_xy = prev_center - curr_center;
                    
                    % Cap nhat vi tri Body
                    body_global_pos(1:2) = body_global_pos(1:2) + shift_xy;
                end
            end
            
            % 4. CAP NHAT LAI VI TRI CHAN TOAN CUC (SAU KHI DA TINH BODY MOI)
            final_feet_global = feet_local + body_global_pos;
            
            % Luu lai cho frame sau
            prev_feet_global = final_feet_global;
            
            % 5. VE ROBOT
            delete(findobj(gca, 'Tag', 'robot_part')); % Xoa frame cu
            
            % Cap nhat mount points theo vi tri toan cuc
            current_mount_pts_global = mount_pts_local + body_global_pos;
            draw_robot(CURRENT_ANGLES, current_mount_pts_global, mount_angles, L_COXA, L_FEMUR, L_TIBIA);
            
            % 6. CAMERA FOLLOW (Camera di chuyen theo Robot)
            cx = body_global_pos(1); cy = body_global_pos(2);
            xlim([cx-400, cx+400]);
            ylim([cy-400, cy+400]);
            
            title({['CYCLE: ' num2str(cycle) ' | STATE: ' STATE_NAMES{s}], ...
                   ['Body Height: ' num2str(body_global_pos(3), '%.1f') ' mm']}, 'FontSize', 12);
            drawnow;
        end
    end
end

%% ========================================================================
%  CAC HAM HO TRO (HELPER FUNCTIONS)
% ========================================================================

% Ham tinh toan FK tra ve vi tri cac dau chan (Local)
function [feet_tips] = calc_fk_all_legs(angles_mat, mount_pts, mount_angles, L1, L2, L3)
    feet_tips = zeros(6, 3);
    for i = 1:6
        th1 = angles_mat(i, 1); th2 = angles_mat(i, 2); th3 = angles_mat(i, 3);
        alpha = mount_angles(i);
        total_rad = deg2rad(alpha + th1);
        
        J1 = mount_pts(i,:); % Goc la mount point
        
        % Vector Coxa
        dx1 = L1*cos(total_rad); dy1 = L1*sin(total_rad); dz1 = L1*sind(th1); % Coxa thuong xoay phang, check lai cong thuc goc
        % Cong thuc chuan hexapod thong thuong:
        % J2 = J1 + [cos(ang)*L1, sin(ang)*L1, 0] (neu coxa phang)
        % Nhung su dung cong thuc cua ban:
        J2 = J1 + [L1*cos(total_rad), L1*sin(total_rad), L1*sind(th1)]; % Gia su coxa co the nang (th1?)
        
        % Vector Femur
        r_femur = L2*cosd(th2); z_femur = L2*sind(th2);
        J3 = J2 + [r_femur*cos(total_rad), r_femur*sin(total_rad), z_femur];
        
        % Vector Tibia
        r_tibia = L3*cosd(th3); z_tibia = L3*sind(th3);
        J4 = J3 + [r_tibia*cos(total_rad), r_tibia*sin(total_rad), z_tibia];
        
        feet_tips(i, :) = J4;
    end
end

% Ham Ve Robot
function draw_robot(angles_mat, mount_pts_global, mount_angles, L1, L2, L3)
    for i = 1:6
        th1 = angles_mat(i, 1); th2 = angles_mat(i, 2); th3 = angles_mat(i, 3);
        alpha = mount_angles(i);
        total_rad = deg2rad(alpha + th1);
        
        % Tinh toan cac khop dua tren Global Mount Points
        J1 = mount_pts_global(i,:);
        J2 = J1 + [L1*cos(total_rad), L1*sin(total_rad), L1*sind(th1)];
        r_femur = L2*cosd(th2); z_femur = L2*sind(th2);
        J3 = J2 + [r_femur*cos(total_rad), r_femur*sin(total_rad), z_femur];
        r_tibia = L3*cosd(th3); z_tibia = L3*sind(th3);
        J4 = J3 + [r_tibia*cos(total_rad), r_tibia*sin(total_rad), z_tibia];
        
        % Ve Chan
        line_pts = [J1; J2; J3; J4];
        cols = lines(6);
        plot3(line_pts(:,1), line_pts(:,2), line_pts(:,3), 'LineWidth', 3, 'Color', cols(i,:), 'Tag', 'robot_part');
        plot3(line_pts(:,1), line_pts(:,2), line_pts(:,3), '.', 'MarkerSize', 15, 'Color', 'k', 'Tag', 'robot_part');
    end
    
    % Ve Than (Noi cac diem mount point global)
    hex_idx = [1, 2, 3, 6, 5, 4, 1];
    fill3(mount_pts_global(hex_idx,1), mount_pts_global(hex_idx,2), mount_pts_global(hex_idx,3), ...
         [0.2 0.2 0.2], 'FaceAlpha', 0.7, 'EdgeColor', 'k', 'LineWidth', 1.5, 'Tag', 'robot_part');
    
    % Ve Dau (Head indicator) de biet huong di chuyen
    head_pos = (mount_pts_global(1,:) + mount_pts_global(4,:))/2;
    quiver3(head_pos(1), head_pos(2), head_pos(3), 100, 0, 0, 'r', 'LineWidth', 3, 'Tag', 'robot_part', 'AutoScale', 'off');
end