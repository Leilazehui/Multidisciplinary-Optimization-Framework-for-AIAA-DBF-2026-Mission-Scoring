clear; clc;

%% ===================== Hyperparameters =====================
AR = linspace(5, 6, 3);		%%Aspect ratio of the wing
b_range = linspace(1.25,1.524,10);		%%range of the length of the wingspan
rho = 1.225;
g = 9.81;

flight_time = 5*60;   %% expected total amount of time for flight mission 2 and 3 
lap_length = 1000;       %% expected total length of the lap

Vb = 22.2;		%%total voltage through the battery
Imax = 100;		 %%maximum current capacity of the battery allowed for the competition
eta = 0.7;		%%estimated efficiency of the avionics system

duck_mass = 0.020;
puck_mass = 0.1701;
duck_len  = 0.065;
L_nose_tail = 0.484;		%%the length of the nose and the tail part of the fuselage

Cd_banner = 0.2351;		%%estimated drag coefficient of the banner
CLmax = 0.693;			%%maximum lift coefficient induced by the aircraft


banner_range = linspace(0.254,4.0,50);		%%a range of the length of the banner 


%% ===================== Ground Mission REFERENCE =====================
%%Tload_min = Inf;
%%for cargo = 1:16
  %%  for pass = max(3,3*cargo):48
    %%    Tload = 15 + 4*(pass-3) + 4*(cargo-1);
      %%  Tload_min = min(Tload_min, Tload);
   %% end
%%end

Tload_min = 25;


%% ===================== MISSION 2 REFERENCE =====================
best_M2_marks = -Inf;

for b = b_range 
    for ar = AR
        S = b^2./ar;
    end
    fuselage_L = 0.85*b;

    for cargo = 1:16
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

            tf = (pass <= 27)*0.145 + (pass > 27)*0.21;  %%thickness of the fuselage
            Af = tf*fuselage_L; %%frontal area
            Cd0 = 0.028 + 0.002*(pass > 27); %%the parasite drag induced by the thickenss of the fuselage
            Aref = Af + 0.12*S; %%reference area for parasite drag, which involes 12-15% of the wing chordline

            perf = cruise_perf(mass,S,Cd0,0.045,Aref,0,...
                rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

            if ~perf.ok; continue; end %%~perf.ok means not ok

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
S_ref = b_ref^2/ar;

for banner_L = banner_range
    for banner_ar = linspace(1, 5, 100)
        banner_ao = banner_ar;
    end 
    banner_h = banner_L/banner_ao;
    banner_mass = 0.2*banner_L*banner_h;
    A_banner = 0.3*banner_L*banner_h;

    mass = 1.2 + banner_mass;   % light but finite aircraft
    [~,Wh] = battery_selector(mass,Vb);
    if isnan(Wh); continue; end

    perf = cruise_perf(mass,S_ref,0.028,0.045,0.12*S_ref,A_banner,...
        rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax);

    if ~perf.ok; continue; end

    laps = floor(perf.V*flight_time/lap_length);
    RAC = 0.05*(b_ref/12) + 0.75;
    best_M3_marks = max(best_M3_marks, laps*banner_L/RAC);
end

%% ===================== OPTIMISATION LOOP =====================
Nkeep = 100;
best = repmat(struct( ...
    'total',-Inf,'b',NaN,'cargo',NaN,'pass',NaN,...
    'laps2',NaN,'laps3',NaN,'speed2',NaN,'speed3',NaN,...
    'battery_Wh',NaN,'banner_L',NaN, 'Tload', NaN, 'mass_M2', NaN, 'mass_M3', NaN, 'AR', NaN, 'banner_ao', NaN),1,Nkeep);

for b = b_range
    l = banner_L;
    o = l*5;
    S = b^2/ar;
    fuselage_L = 0.85*b;
    if fuselage_L > 1.524; continue; end

    for cargo = 1:16
        for pass = 3:48
            if pass < 3*cargo; continue; end
            if ~payload_fits(pass,b); continue; end

            %% Mission 1
            Tload = 25 + 8*(pass-3) + 8*(cargo-1);
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
                for banner_ar = linspace(1, 5, 100)
                    banner_ao = banner_ar;
                end 
                banner_h = banner_L./banner_ao;
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

            total = 1 + M1 + M2 + M3;

            candidate = struct('total',total,'b',b,'cargo',cargo,...
                'pass',pass,'laps2',laps2,'laps3',best_laps3,...
                'speed2',perf2.V,'speed3',best_V3,...
                'battery_Wh',Wh,'banner_L',best_banner, 'Tload', Tload, 'mass_M2', mass_M2, 'mass_M3', mass_M3, 'AR', ar, 'banner_ao', banner_ao);

            for k = 1:Nkeep
                if candidate.total > best(k).total
                    best = [best(1:k-1),candidate,best(k:end-1)];
                    break
                end
            end
        end
    end
end

%% ===================== RESULTS DEMONSTRATED IN TABLES =====================
fprintf('\nTOP 25 AIRCRAFT CONFIGURATIONS\n');

for i = 1:100
    fprintf('\nCASE #%d\n',i);
    fprintf(' Total Score     : %.3f\n', best(i).total);
    fprintf(' Wingspan (m)    : %.3f\n', best(i).b);
    fprintf(' Cargo (pucks)  : %d\n', best(i).cargo);
    fprintf(' Passengers     : %d\n', best(i).pass);
    fprintf(' Mission 2 laps : %d @ %.1f m/s\n', best(i).laps2, best(i).speed2);
    fprintf(' Mission 3 laps : %d @ %.1f m/s\n', best(i).laps3, best(i).speed3);
    fprintf(' Battery (Wh)   : %.1f\n', best(i).battery_Wh);
    fprintf(' Banner length (m): %.2f\n', best(i).banner_L);
    fprintf(' Ground mission time: %.2f\n', best(i).Tload);
    fprintf(' mass_M2: %.1f Kg\n', best(i).mass_M2);
    fprintf(' mass_M3: %.1f Kg\n', best(i).mass_M3);
    fprintf(' AR: %.1f \n', best(i).AR);
    fprintf(' banner_ao: %.1f \n', best(i).banner_ao);
end

disp('BEST SINGLE-AIRCRAFT CONFIGURATION');
disp(best);

%% ===================== FUNCTIONS FOR HYPERPARAMETERS =====================
function W = empty_weight_selector(pass,cargo)
    if pass <=3 && cargo <= 1
        W = 2.02;
    elseif pass < 10 && pass > 3 && cargo <= 3 && cargo > 1
        W = 2.9;
    elseif pass < 14 && pass > 10 && cargo <= 4 && cargo > 3
        W = 3.2;
    elseif pass <= 27 && pass > 14 && cargo <= 9 && cargo > 4
        W = 3.0;
    elseif pass <= 39 && pass > 27 && cargo <= 13 && cargo > 9 
        W = 3.5;
    elseif pass <= 42 && pass > 39 && cargo <= 14 && cargo > 13
        W = 3.5;
    elseif pass <= 48 && pass > 42 && cargo <= 16 && cargo > 14
        W = 4.3;
    else
        W = NaN;
    end
end

function [mAh,Wh] = battery_selector(mass,V)
    if mass <2.0, mAh = 3400;
    elseif mass < 3.0 && mass > 2.0, mAh = 3400;
    elseif mass < 3.5 && mass > 3.0, mAh = 3600;
    elseif mass < 5.0 && mass > 3.5, mAh = 3800;
    elseif mass < 6.5 && mass > 5.0, mAh = 4200;
    elseif mass < 8.0 && mass > 6.5, mAh = 4800;
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
    rho,eta,Vb,Imax,Wh,flight_time,Cd_banner,CLmax,~)

    W = mass*9.81;
    Pmax = Vb*Imax*eta;

    V = linspace(15,32,800);
    D = 0.5*rho*V.^2.*Cd0.*Aref + k*(W^2)./(0.5*rho.*S.*V.^2);
    if A_banner>0
        D = D + 0.5*rho*V.^2*Cd_banner*A_banner;
    end

    P = D.*V/eta;
    CL = W./(0.5*rho*V.^2.*S);
    E = P*(flight_time/3600);

    valid = (P<=Pmax)&(CL<=CLmax)&(E<=Wh);
    if ~any(valid), perf.ok=false; return; end

    perf.ok=true;
    perf.V=max(V(valid));
end
