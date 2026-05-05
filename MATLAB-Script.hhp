clear; clc;

%% ===================== CONSTANTS =====================
AR = 5.5;
b_range = linspace(1.25,1.45,10);
rho = 1.225;
g = 9.81;

flight_time = 3.5*60;   % s
lap_length = 950;       % m

Vb = 22.2;
Imax = 100;
eta = 0.7;

duck_mass = 0.020;
puck_mass = 0.1701;
duck_len  = 0.065;
L_nose_tail = 0.484;

Cd_banner = 1.4;
CLmax = 1.2;

banner_range = linspace(0.254,4.0,30);

%% ===================== MISSION 1 REFERENCE =====================
Tload_min = Inf;
for cargo = 1:12
    for pass = max(3,3*cargo):48
        Tload = 10 + 6*(pass-3) + 6*(cargo-1);
        Tload_min = min(Tload_min, Tload);
    end
end

%% ===================== MISSION 2 REFERENCE =====================
best_M2_marks = -Inf;

for b = b_range
    S = b^2/AR;
    fuselage_L = 0.85*b;

    for cargo = 1:12
        for pass = 3:48
            if pass < 3*cargo; continue; end
            if ~payload_fits(pass,b); continue; end
            if fuselage_L > 1.524; continue; end

            W_empty = empty_weight_selector(pass,cargo);
            if isnan(W_empty); continue; end

            payload_mass = pass*duck_mass + cargo*puck_mass;
            mass = W_empty + payload_mass;

            [~,Wh] = battery_selector(mass,Vb);
            if isnan(Wh); continue; end

            tf = (pass <= 27)*0.145 + (pass > 27)*0.21;
            Af = tf*fuselage_L;
            Cd0 = 0.028 + 0.002*(pass > 27);
            Aref = Af + 0.12*S;

            perf = cruise_perf(mass,S,Cd0,0.045,Aref,0,...
                rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

            if ~perf.ok; continue; end

            laps = floor(perf.V*flight_time/lap_length);
            income = pass*(6+2*laps) + cargo*(10+8*laps);
            cost = laps*(10+0.5*pass+2*cargo)*(Wh/100);
            best_M2_marks = max(best_M2_marks, income-cost);
        end
    end
end

%% ===================== MISSION 3 REFERENCE =====================
best_M3_marks = 0;

b_ref = max(b_range);
S_ref = b_ref^2/AR;

for banner_L = banner_range
    banner_h = banner_L/5;
    banner_mass = 0.2*banner_L*banner_h;
    A_banner = 0.3*banner_L*banner_h;

    mass = 2.2 + banner_mass;   % light but finite aircraft
    [~,Wh] = battery_selector(mass,Vb);
    if isnan(Wh); continue; end

    perf = cruise_perf(mass,S_ref,0.028,0.045,0.12*S_ref,A_banner,...
        rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

    if ~perf.ok; continue; end

    laps = floor(perf.V*flight_time/lap_length);
    RAC = 0.05*(b_ref/12) + 0.75;
    best_M3_marks = max(best_M3_marks, laps*banner_L/RAC);
end

%% ===================== OPTIMISATION =====================
Nkeep = 3;
best = repmat(struct( ...
    'total',-Inf,'b',NaN,'cargo',NaN,'pass',NaN,...
    'laps2',NaN,'laps3',NaN,'speed2',NaN,'speed3',NaN,...
    'battery_Wh',NaN,'banner_L',NaN),1,Nkeep);

for b = b_range
    S = b^2/AR;
    fuselage_L = 0.85*b;
    if fuselage_L > 1.524; continue; end

    for cargo = 1:12
        for pass = 3:48
            if pass < 3*cargo; continue; end
            if ~payload_fits(pass,b); continue; end

            %% Mission 1
            Tload = 10 + 6*(pass-3) + 6*(cargo-1);
            M1 = Tload_min / Tload;

            %% Geometry & mass
            W_empty = empty_weight_selector(pass,cargo);
            if isnan(W_empty); continue; end

            payload_mass = pass*duck_mass + cargo*puck_mass;
            mass_M2 = W_empty + payload_mass;

            [~,Wh] = battery_selector(mass_M2,Vb);
            if isnan(Wh); continue; end

            tf = (pass <= 27)*0.145 + (pass > 27)*0.21;
            Af = tf*fuselage_L;
            Cd0 = 0.028 + 0.002*(pass > 27);
            Aref = Af + 0.12*S;

            %% Mission 2
            perf2 = cruise_perf(mass_M2,S,Cd0,0.045,Aref,0,...
                rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

            if ~perf2.ok; continue; end

            laps2 = floor(perf2.V*flight_time/lap_length);
            income = pass*(6+2*laps2) + cargo*(10+8*laps2);
            cost = laps2*(10+0.5*pass+2*cargo)*(Wh/100);
            M2 = 1 + (income-cost)/best_M2_marks;

            %% Mission 3 (banner optimised)
            M3 = -Inf;
            for banner_L = banner_range
                banner_h = banner_L/5;
                banner_mass = 0.2*banner_L*banner_h;
                A_banner = 0.3*banner_L*banner_h;

                mass_M3 = W_empty + banner_mass;

                perf3 = cruise_perf(mass_M3,S,Cd0,0.045,Aref,A_banner,...
                    rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

                if ~perf3.ok; continue; end

                laps3 = floor(perf3.V*flight_time/lap_length);
                RAC = 0.05*(b/12) + 0.75;
                score = 2 + (laps3*banner_L/RAC)/best_M3_marks;

                if score > M3
                    M3 = score;
                    best_banner = banner_L;
                    best_laps3 = laps3;
                    best_V3 = perf3.V;
                end
            end

            total = M1 + M2 + M3;

            candidate = struct('total',total,'b',b,'cargo',cargo,...
                'pass',pass,'laps2',laps2,'laps3',best_laps3,...
                'speed2',perf2.V,'speed3',best_V3,...
                'battery_Wh',Wh,'banner_L',best_banner);

            for k = 1:Nkeep
                if candidate.total > best(k).total
                    best = [best(1:k-1),candidate,best(k:end-1)];
                    break
                end
            end
        end
    end
end

%% ===================== RESULTS =====================
disp('TOP 3 CONFIGURATIONS'); disp(best);

%% ===================== FUNCTIONS =====================
function W = empty_weight_selector(pass,cargo)
    if pass < 14 && cargo <= 4
        W = 2.0;
    elseif pass <= 27 && cargo <= 9
        W = 3.0;
    elseif pass <= 39 && cargo <= 13
        W = 3.5;
    elseif pass <= 42 && cargo <= 14
        W = 4.0;
    elseif pass <= 48 && cargo <= 16
        W = 4.3;
    else
        W = NaN;
    end
end

function [mAh,Wh] = battery_selector(mass,V)
    if mass < 3, mAh = 3400;
    elseif mass < 3.5, mAh = 3600;
    elseif mass < 5, mAh = 3800;
    elseif mass < 6.5, mAh = 4200;
    elseif mass < 8, mAh = 4800;
    else, mAh = NaN;
    end
    Wh = V*mAh/1000;
end

function ok = payload_fits(pass,b)
    L_payload = 0.85*b - 0.484;
    if L_payload <= 0, ok=false; return; end
    ducks_per_row = (pass<=18)*2 + (pass>18)*3;
    ok = ceil(pass/ducks_per_row)*0.065 <= L_payload;
end

function perf = cruise_perf(mass,S,Cd0,k,Aref,A_banner,...
    rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax)

    W = mass*9.81;
    Pmax = Vb*Imax*eta;

    V = linspace(15,32,800);
    D = 0.5*rho*V.^2.*Cd0.*Aref + k*(W^2)./(0.5*rho*S.*V.^2);
    if A_banner>0
        D = D + 0.5*rho*V.^2*Cd_banner*A_banner;
    end

    P = D.*V/eta;
    CL = W./(0.5*rho*V.^2*S);
    E = P*(flight_time/3600);

    valid = (P<=Pmax)&(CL<=CLmax)&(E<=Wh);
    if ~any(valid), perf.ok=false; return; end

    perf.ok=true;
    perf.V=max(V(valid));
end
