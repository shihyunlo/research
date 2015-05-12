function [PW normP normW y_CT y_CQ y_v] = propGPtraining(cov, scaling)


DATA = data_compute();

%% data training
% close all
% cov = [0.5 2.58 0.2];
% cov = [-0.6553 -1.2025 4.4315];
% cov = [0.1 1.05 0.2];
% cov = [3.3337 6.1250 1.9653];
% variance = 10;
CT_vol = DATA(:,1) - 6.189e-8;
CQ_vol = DATA(:,2);
v = DATA(:,3);
P = DATA(:,4);
W = DATA(:,5);
normP = max(P);
P = P/normP;
normW = max(W);
W = W/normW;
PW = [P W];
KX = covSEard(cov, PW*scaling, PW*scaling);
y_CT = KX\CT_vol;
y_CQ = KX\CQ_vol;
y_v = KX\v;


%% data validation, table generation
% ds1 = 50;
% ds2 = 50;
% shortMAX = [data_static(5,4) data_static(9,4) data_static(12,4)];
% longMAX = [data_static(9,4) data_static(12,4) data_static(16,4)];
% shortMIN = [data_wind{1}(end,4) data_wind{2}(end,4) data_wind{3}(end,4)];
% longMIN = [data_wind{2}(end,4) data_wind{3}(end,4) data_wind{4}(end,4)];
% short_rpms = [4007 5012 6018];
% long_rpms = [5012 6018 7013];
% 
% IV = [];
% ICT = [];
% PG = [];
% WG = [];
% P_grid = [];
% 
% for j = 1:3
%     
%     short_max = shortMAX(j);%around rpm = 4000
%     long_max = longMAX(j);%around rpm = 7000
%     short_min = shortMIN(j);
%     long_min = longMIN(j);
%     P_shortgrid = short_min:(short_max - short_min)/ds1:short_max;
%     P_longgrid = long_min:(long_max-long_min)/ds1:long_max;
% %     P_grid = zeros(length(P_shortgrid));
% 
%     for i = 1:length(P_shortgrid)
%         P_new = P_shortgrid(i):(P_longgrid(i)-P_shortgrid(i))/ds2:P_longgrid(i);
%         P_grid(i,:) = P_new;
%     end
%     PG = [PG P_grid];
% 
%     P_sort = P_grid(1,:);
%     Pstep = (P_grid(end,:)-P_sort)/ds1;
% 
%     Wstep = (long_rpms(j)-short_rpms(j))/ds2;
% 
%     W_grid = short_rpms(j):Wstep:long_rpms(j);
%     W_grid = repmat(W_grid,size(P_grid,1),1);
%     WG = [WG W_grid];
% 
%     P_data = reshape(P_grid,[],1);
%     W_data = reshape(W_grid,[],1);
%     PW_table= [P_data/normP W_data/normW];
% 
%     k = covSEard(cov, PW, PW_table);
%     CT_table = k'*y_CT;
%     CQ_table = k'*y_CQ;
%     v_table = k'*y_v;
% 
%     image_CT = reshape(CT_table,[],ds1+1);
%     image_CQ = reshape(CQ_table,[],ds1+1);
%     image_v = reshape(v_table,[],ds1+1);
%     IV = [IV image_v];
%     ICT = [ICT image_CT];
% end
% plot(v_table)
% figure
% imagesc(IV)
% figure
% imagesc(ICT)
% figure
% imagesc(PG)
% figure
% imagesc(WG)

% hyp.cov = [3.3337 6.1250 1.9653];
% hyp.lik = 1.6039;
% meanfunc = @meanZero;
% covfunc = @covSEard;
% likfunc = @likGauss;
% 
% hyp = minimize(hyp, @gp, -100, @infExact, [], covfunc, likfunc, PW, v);