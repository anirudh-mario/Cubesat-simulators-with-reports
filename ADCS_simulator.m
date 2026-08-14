%% =======================================================================
%  Interactive CubeSat ADCS Simulation
% =======================================================================
%  Drag the Yaw / Pitch / Roll sliders at ANY time while the simulation
%  is running -> the target orientation updates live -> the PD attitude
%  controller (reaction-wheel torque, saturated) drives the CubeSat to
%  track it in real time, just like a real ADCS closed control loop.
%
%  Buttons:
%    "Randomize Target"  -> jumps to a random commanded attitude
%    "Disturb (Tumble)"  -> kicks in a random angular rate disturbance
%    "Reset"              -> zeros target & rates, re-centers CubeSat
%
%  Close the figure window to stop the simulation loop.
%
%  Requires: base MATLAB only (no toolboxes).
% =======================================================================

clear; close all; clc;

%% ---------------------- PHYSICAL / CONTROL PARAMETERS ------------------
sat.dims = [0.10 0.10 0.30];        % 3U CubeSat dims (m)
sat.mass = 4.0;                     % kg

Ix = (sat.mass/12)*(sat.dims(2)^2 + sat.dims(3)^2);
Iy = (sat.mass/12)*(sat.dims(1)^2 + sat.dims(3)^2);
Iz = (sat.mass/12)*(sat.dims(1)^2 + sat.dims(2)^2);
I    = diag([Ix Iy Iz]);
Iinv = inv(I);

Kp      = 0.015;      % proportional gain
Kd      = 0.05;       % derivative (damping) gain
tau_max = 0.004;       % N*m, actuator saturation

dt        = 0.03;      % s, sim step
win_T     = 18;         % s, scrolling telemetry window length
buf_N     = round(win_T/dt);

%% ---------------------------- STATE -------------------------------------
q = [1;0;0;0];                       % current attitude quaternion
w = deg2rad([8;-5;10]);              % start with a small initial tumble

target_eul_deg = [0 0 0];            % [yaw pitch roll], set by sliders
q_target = eul2quat_wxyz(deg2rad(target_eul_deg));

% Scrolling telemetry buffers
tbuf   = nan(1,buf_N);
wbuf   = nan(3,buf_N);
errbuf = nan(1,buf_N);
taubuf = nan(3,buf_N);
tnow   = 0;

%% ============================ FIGURE / UI ===============================
fig = figure('Color','w','Position',[60 60 1250 700], ...
             'Name','Interactive CubeSat ADCS','NumberTitle','off');

% ---- 3D CubeSat view -----------------------------------------------
ax3d = subplot('Position',[0.05 0.30 0.42 0.62]);
axis(ax3d,'equal'); grid(ax3d,'on'); hold(ax3d,'on');
xlim(ax3d,[-0.35 0.35]); ylim(ax3d,[-0.35 0.35]); zlim(ax3d,[-0.35 0.35]);
xlabel(ax3d,'X'); ylabel(ax3d,'Y'); zlabel(ax3d,'Z');
view(ax3d,135,20);
camlight(ax3d,'headlight'); lighting(ax3d,'gouraud');
titleH = title(ax3d,'CubeSat Attitude');

d = sat.dims/2;
verts0 = [-d(1) -d(2) -d(3);  d(1) -d(2) -d(3);  d(1)  d(2) -d(3); -d(1)  d(2) -d(3); ...
          -d(1) -d(2)  d(3);  d(1) -d(2)  d(3);  d(1)  d(2)  d(3); -d(1)  d(2)  d(3)];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
satPatch = patch('Faces',faces,'Vertices',verts0,'FaceColor',[0.2 0.55 0.9], ...
                  'FaceAlpha',0.85,'EdgeColor','k','Parent',ax3d);

axLen = 0.28;
hX = plot3(ax3d,[0 axLen],[0 0],[0 0],'r-','LineWidth',2.5);
hY = plot3(ax3d,[0 0],[0 axLen],[0 0],'g-','LineWidth',2.5);
hZ = plot3(ax3d,[0 0],[0 0],[0 axLen],'b-','LineWidth',2.5);

% Target axes (dashed) - these move whenever the user changes the sliders
htX = plot3(ax3d,[0 axLen],[0 0],[0 0],'r--','LineWidth',1.3);
htY = plot3(ax3d,[0 0],[0 axLen],[0 0],'g--','LineWidth',1.3);
htZ = plot3(ax3d,[0 0],[0 0],[0 axLen],'b--','LineWidth',1.3);
legend(ax3d,[hX hY hZ htX],{'Body X','Body Y','Body Z','Target axes (dashed)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',8);

% ---- Telemetry (scrolling) ------------------------------------------
axW = subplot('Position',[0.56 0.62 0.40 0.30]); hold(axW,'on'); grid(axW,'on');
ylabel(axW,'Angular rate (deg/s)'); title(axW,'Body Angular Velocity');
lw1 = plot(axW,tbuf,wbuf(1,:),'r','LineWidth',1.4);
lw2 = plot(axW,tbuf,wbuf(2,:),'g','LineWidth',1.4);
lw3 = plot(axW,tbuf,wbuf(3,:),'b','LineWidth',1.4);
legend(axW,{'\omega_x','\omega_y','\omega_z'},'Location','eastoutside');

axE = subplot('Position',[0.56 0.30 0.40 0.24]); hold(axE,'on'); grid(axE,'on');
ylabel(axE,'|q_{err}|'); title(axE,'Attitude Error (0 = target reached)');
le1 = plot(axE,tbuf,errbuf,'k','LineWidth',1.6);
ylim(axE,[0 1]);

% ---- Sliders: Yaw / Pitch / Roll target (deg) ------------------------
uicontrol('Style','text','String','TARGET ORIENTATION (drag anytime)', ...
          'FontWeight','bold','Units','normalized','Position',[0.05 0.235 0.42 0.03], ...
          'BackgroundColor','w','FontSize',10);

sYaw = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0, ...
          'Units','normalized','Position',[0.11 0.185 0.30 0.03]);
uicontrol('Style','text','String','Yaw','Units','normalized', ...
          'Position',[0.05 0.180 0.05 0.03],'BackgroundColor','w');
txtYaw = uicontrol('Style','text','String','0 deg','Units','normalized', ...
          'Position',[0.42 0.180 0.06 0.03],'BackgroundColor','w');

sPitch = uicontrol('Style','slider','Min',-90,'Max',90,'Value',0, ...
          'Units','normalized','Position',[0.11 0.140 0.30 0.03]);
uicontrol('Style','text','String','Pitch','Units','normalized', ...
          'Position',[0.05 0.135 0.05 0.03],'BackgroundColor','w');
txtPitch = uicontrol('Style','text','String','0 deg','Units','normalized', ...
          'Position',[0.42 0.135 0.06 0.03],'BackgroundColor','w');

sRoll = uicontrol('Style','slider','Min',-180,'Max',180,'Value',0, ...
          'Units','normalized','Position',[0.11 0.095 0.30 0.03]);
uicontrol('Style','text','String','Roll','Units','normalized', ...
          'Position',[0.05 0.090 0.05 0.03],'BackgroundColor','w');
txtRoll = uicontrol('Style','text','String','0 deg','Units','normalized', ...
          'Position',[0.42 0.090 0.06 0.03],'BackgroundColor','w');

% ---- Buttons -----------------------------------------------------------
btnRand = uicontrol('Style','pushbutton','String','Randomize Target', ...
          'Units','normalized','Position',[0.05 0.02 0.13 0.045],'FontSize',9);
btnTumble = uicontrol('Style','pushbutton','String','Disturb (Tumble)', ...
          'Units','normalized','Position',[0.19 0.02 0.13 0.045],'FontSize',9);
btnReset = uicontrol('Style','pushbutton','String','Reset', ...
          'Units','normalized','Position',[0.33 0.02 0.10 0.045],'FontSize',9);

statusTxt = uicontrol('Style','text','String','Status: tracking target...', ...
          'Units','normalized','Position',[0.56 0.02 0.40 0.045], ...
          'BackgroundColor','w','FontSize',9,'ForegroundColor',[0 0.45 0]);

% Button callbacks just set flags / values; main loop below reads them.
setappdata(fig,'doRandom',false);
setappdata(fig,'doTumble',false);
setappdata(fig,'doReset',false);
btnRand.Callback   = @(~,~) setappdata(fig,'doRandom',true);
btnTumble.Callback = @(~,~) setappdata(fig,'doTumble',true);
btnReset.Callback  = @(~,~) setappdata(fig,'doReset',true);

%% ============================ MAIN LOOP =================================
while ishandle(fig)

    % ---- Handle button presses ------------------------------------
    if getappdata(fig,'doRandom')
        set(sYaw,'Value',(rand*2-1)*180);
        set(sPitch,'Value',(rand*2-1)*80);
        set(sRoll,'Value',(rand*2-1)*180);
        setappdata(fig,'doRandom',false);
        set(statusTxt,'String','Status: new random target set','ForegroundColor',[0.8 0.4 0]);
    end
    if getappdata(fig,'doTumble')
        w = w + deg2rad((rand(3,1)*2-1)*35);   % random rate kick
        setappdata(fig,'doTumble',false);
        set(statusTxt,'String','Status: disturbance applied - recovering...','ForegroundColor',[0.8 0 0]);
    end
    if getappdata(fig,'doReset')
        q = [1;0;0;0]; w = [0;0;0];
        set(sYaw,'Value',0); set(sPitch,'Value',0); set(sRoll,'Value',0);
        tbuf(:) = nan; wbuf(:) = nan; errbuf(:) = nan; taubuf(:) = nan; tnow = 0;
        setappdata(fig,'doReset',false);
        set(statusTxt,'String','Status: reset','ForegroundColor',[0 0 0.6]);
    end

    % ---- Read target sliders (live) --------------------------------
    yaw_deg   = get(sYaw,'Value');
    pitch_deg = get(sPitch,'Value');
    roll_deg  = get(sRoll,'Value');
    set(txtYaw,'String',sprintf('%.0f deg',yaw_deg));
    set(txtPitch,'String',sprintf('%.0f deg',pitch_deg));
    set(txtRoll,'String',sprintf('%.0f deg',roll_deg));
    q_target = eul2quat_wxyz(deg2rad([yaw_deg pitch_deg roll_deg]));

    % ---- Controller: PD on quaternion error + rate -------------------
    q_err = quatMult(quatConj(q_target), q);
    if q_err(1) < 0, q_err = -q_err; end
    tau_cmd = -Kp*q_err(2:4) - Kd*w;
    tau_cmd = max(min(tau_cmd, tau_max), -tau_max);

    % ---- Rigid-body dynamics + quaternion kinematics (Euler integ.) --
    wdot = Iinv*(tau_cmd - cross(w, I*w));
    qdot = 0.5*quatMult(q, [0; w]);
    w = w + wdot*dt;
    q = q + qdot*dt; q = q/norm(q);

    tnow = tnow + dt;

    % ---- Update scrolling telemetry buffers ---------------------------
    tbuf   = [tbuf(2:end)   tnow];
    wbuf   = [wbuf(:,2:end) rad2deg(w)];
    errbuf = [errbuf(2:end) norm(q_err(2:4))];
    taubuf = [taubuf(:,2:end) tau_cmd];

    err_now = norm(q_err(2:4));
    if err_now < 0.01
        set(statusTxt,'String','Status: TARGET LOCKED ✓','ForegroundColor',[0 0.5 0]);
    end

    % ---- Redraw 3D CubeSat ---------------------------------------------
    R = quat2rotm_wxyz(q);
    set(satPatch,'Vertices',(R*verts0')');
    bx = R*[axLen;0;0]; by = R*[0;axLen;0]; bz = R*[0;0;axLen];
    set(hX,'XData',[0 bx(1)],'YData',[0 bx(2)],'ZData',[0 bx(3)]);
    set(hY,'XData',[0 by(1)],'YData',[0 by(2)],'ZData',[0 by(3)]);
    set(hZ,'XData',[0 bz(1)],'YData',[0 bz(2)],'ZData',[0 bz(3)]);

    Rt = quat2rotm_wxyz(q_target);
    tx = Rt*[axLen;0;0]; ty = Rt*[0;axLen;0]; tz = Rt*[0;0;axLen];
    set(htX,'XData',[0 tx(1)],'YData',[0 tx(2)],'ZData',[0 tx(3)]);
    set(htY,'XData',[0 ty(1)],'YData',[0 ty(2)],'ZData',[0 ty(3)]);
    set(htZ,'XData',[0 tz(1)],'YData',[0 tz(2)],'ZData',[0 tz(3)]);

    set(titleH,'String',sprintf('CubeSat Attitude   t=%.1fs   |err|=%.3f',tnow,err_now));

    % ---- Update telemetry plots ----------------------------------------
    set(lw1,'XData',tbuf,'YData',wbuf(1,:));
    set(lw2,'XData',tbuf,'YData',wbuf(2,:));
    set(lw3,'XData',tbuf,'YData',wbuf(3,:));
    xlim(axW,[max(0,tnow-win_T) max(win_T,tnow)]);

    set(le1,'XData',tbuf,'YData',errbuf);
    xlim(axE,[max(0,tnow-win_T) max(win_T,tnow)]);

    drawnow limitrate;
end

fprintf('Simulation window closed.\n');

%% ============================ HELPER FUNCTIONS ==========================
function q = quatMult(a,b)
    aw=a(1); ax=a(2); ay=a(3); az=a(4);
    bw=b(1); bx=b(2); by=b(3); bz=b(4);
    q = [aw*bw - ax*bx - ay*by - az*bz;
         aw*bx + ax*bw + ay*bz - az*by;
         aw*by - ax*bz + ay*bw + az*bx;
         aw*bz + ax*by - ay*bx + az*bw];
end

function qc = quatConj(q)
    qc = [q(1); -q(2); -q(3); -q(4)];
end

function R = quat2rotm_wxyz(q)
    q = q/norm(q);
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2)   2*(x*y - z*w)   2*(x*z + y*w);
         2*(x*y + z*w)   1-2*(x^2+z^2)   2*(y*z - x*w);
         2*(x*z - y*w)   2*(y*z + x*w)   1-2*(x^2+y^2)];
end

function q = eul2quat_wxyz(eul)
    yaw=eul(1); pitch=eul(2); roll=eul(3);
    cy=cos(yaw/2); sy=sin(yaw/2);
    cp=cos(pitch/2); sp=sin(pitch/2);
    cr=cos(roll/2); sr=sin(roll/2);
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
    q = q/norm(q);
end
