function [DATA] = data_compute()

% to acquire the data as the format of CT CQ wind_speed power rpm
%load propeller data, 8x3.8

%% static: RPM kT kP
static = [
3030   0.0923   0.0415
3226   0.0935   0.0415
3555   0.0944   0.0418
3808   0.0948   0.0420
4093   0.0965   0.0424
4337   0.0968   0.0425
4619   0.0987   0.0432
4861   0.0995   0.0437
5132   0.1004   0.0441
5411   0.1017   0.0447
5676   0.1030   0.0452
5946   0.1045   0.0460
6212   0.1055   0.0464
6484   0.1073   0.0471
6731   0.1092   0.0477
7015   0.1109   0.0481];
static = [static zeros(length(static), 1)];

wind = cell(5,1);
%% W = 4007: J kT kP eta
wind{1} = [0.174   0.0777   0.0409   0.330
0.218   0.0713   0.0401   0.387
0.263   0.0637   0.0387   0.433
0.307   0.0566   0.0375   0.463
0.355   0.0478   0.0357   0.476
0.405   0.0379   0.0338   0.454
0.458   0.0267   0.0316   0.386
0.497   0.0175   0.0294   0.295
0.547   0.0043   0.0260   0.091
0.596   -0.0079   0.0230   -0.204
0.631   -0.0173   0.0205   -0.530
0.683   -0.0328   0.0164   -1.364
0.725   -0.0468   0.0131   -2.586
0.777   -0.0635   0.0087   -5.654
0.808   -0.0737   0.0061   -9.791];

%% W = 5012: J kT kP eta
wind{2} = [0.138   0.0849   0.0428   0.274
0.176   0.0796   0.0422   0.331
0.209   0.0747   0.0416   0.376
0.245   0.0688   0.0406   0.416
0.289   0.0616   0.0393   0.452
0.323   0.0556   0.0382   0.470
0.358   0.0493   0.0370   0.477
0.399   0.0411   0.0353   0.464
0.432   0.0343   0.0339   0.437
0.475   0.0244   0.0317   0.366
0.508   0.0162   0.0297   0.278
0.544   0.0070   0.0274   0.138
0.581   -0.0031   0.0246   -0.073
0.618   -0.0130   0.0220   -0.364
0.657   -0.0235   0.0193   -0.802
0.684   -0.0318   0.0170   -1.283
0.728   -0.0464   0.0132   -2.554];

%% W = 6018: J kT kP eta
wind{3} = [
0.113   0.0920   0.0449   0.232
0.142   0.0879   0.0444   0.282
0.174   0.0834   0.0439   0.330
0.204   0.0787   0.0432   0.371
0.234   0.0741   0.0425   0.408
0.266   0.0688   0.0417   0.439
0.298   0.0633   0.0407   0.464
0.330   0.0574   0.0395   0.480
0.361   0.0516   0.0383   0.486
0.390   0.0458   0.0371   0.482
0.422   0.0388   0.0355   0.461
0.451   0.0324   0.0340   0.429
0.484   0.0244   0.0320   0.368
0.509   0.0182   0.0306   0.304
0.541   0.0097   0.0284   0.185
0.566   0.0030   0.0265   0.063
0.606   -0.0080   0.0235   -0.207];
%% W = 7013: J kT kP eta
wind{4} = [0.097   0.0992   0.0467   0.206
0.122   0.0954   0.0461   0.253
0.151   0.0914   0.0456   0.303
0.174   0.0881   0.0452   0.339
0.203   0.0832   0.0444   0.381
0.228   0.0794   0.0438   0.413
0.255   0.0749   0.0431   0.443
0.281   0.0703   0.0423   0.467
0.306   0.0657   0.0413   0.486
0.333   0.0608   0.0403   0.501
0.361   0.0552   0.0391   0.510
0.386   0.0502   0.0381   0.509
0.412   0.0447   0.0368   0.500
0.433   0.0399   0.0356   0.485
0.462   0.0330   0.0339   0.450
0.487   0.0269   0.0324   0.404
0.516   0.0192   0.0305   0.324];

%% W = 7018: J kT kP eta
wind{5} = [0.434   0.0396   0.0359   0.479
0.461   0.0333   0.0343   0.448
0.490   0.0263   0.0326   0.396
0.513   0.0202   0.0311   0.334
0.545   0.0114   0.0289   0.216
0.569   0.0042   0.0270   0.089
0.592   -0.0027   0.0251   -0.064
0.622   -0.0110   0.0228   -0.301
0.646   -0.0181   0.0210   -0.558
0.671   -0.0253   0.0190   -0.893
0.700   -0.0353   0.0165   -1.499
0.723   -0.0427   0.0145   -2.124
0.752   -0.0527   0.0121   -3.273
0.774   -0.0599   0.0102   -4.543
0.798   -0.0691   0.0082   -6.734];

%% convert data into CT CQ wind_speed Power rpm
% CT = kT rho D^4 / (60)^2
% CQ = kQ rho D^5 / (60)^2
% J = u/Dn, u(wind_speed) = J*D*n = J*D*w/60
% P(power in)= 2pi n Q = 2pi w/60 CQ w^2

rho = 1.225;
D = 0.2032;
data_static = [static(:,2)*rho*D^4/(60)^2 static(:,3)*rho*D^5/(60)^2];
data_static = [data_static zeros(length(data_static),1) data_static(:,2).*static(:,1).^3*2*pi/60 ...
    static(:,1)];
data_wind = cell(5,1);
DATA = data_static;
for i = 1:5
    data = wind{i};
    J = data(:,1);
    kT = data(:,2);
    kQ = data(:,3);
    if i == 1
        w = 4007;
    elseif i == 2
        w = 5012;
    elseif i == 3
        w = 6018;
    elseif i == 4
        w = 7013;
    elseif i == 5
        w = 7018;
    end
    data_wind{i} = [kT*rho*D^4/(60)^2 kQ*rho*D^5/(60)^2 J*D*w/60 ...
        kQ*rho*D^5/(60)^2*w^3/60*2*pi w*ones(length(data),1)];
    DATA = [DATA;data_wind{i}];
end


end



