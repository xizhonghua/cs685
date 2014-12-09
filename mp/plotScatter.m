ctl = load('bestctl.txt');

hFig = figure();
set(hFig, 'Position', [50 20 800 640])


% K_rho
h = scatter3(ctl(:,5),ctl(:,6),ctl(:,7),4,ctl(:,1)); colorbar();
F = getframe(gcf); imwrite(F.cdata,'k_rho_3d.png','png');
view([0,0,1]); daspect([1,1,1]);axis([-15,15,-15,15])
F = getframe(gcf); imwrite(F.cdata,'k_rho_proj.png','png');

% k_alpha
h = scatter3(ctl(:,5),ctl(:,6),ctl(:,7),4,ctl(:,2)); colorbar();
F = getframe(gcf); imwrite(F.cdata,'k_alpha_3d.png','png');
view([0,0,1]); daspect([1,1,1]);axis([-15,15,-15,15])
F = getframe(gcf); imwrite(F.cdata,'k_alpha_proj.png','png');

% k_beta
h = scatter3(ctl(:,5),ctl(:,6),ctl(:,7),4,ctl(:,3)); colorbar();
F = getframe(gcf); imwrite(F.cdata,'k_beta_3d.png','png');
view([0,0,1]); daspect([1,1,1]);axis([-15,15,-15,15])
F = getframe(gcf); imwrite(F.cdata,'k_beta_proj.png','png');

% svm train
svm = load('train_svm.txt');
h = scatter3(svm(:,1),svm(:,2),svm(:,3),4,svm(:,4)); colorbar();
F = getframe(gcf); imwrite(F.cdata,'svm_train_3d.png','png');
view([0,0,1]); daspect([1,1,1]);axis([-15,15,-15,15])
F = getframe(gcf); imwrite(F.cdata,'svm_train_proj.png','png');

% svm test
svm = load('test_svm.txt');
h = scatter3(svm(:,1),svm(:,2),svm(:,3),4,svm(:,4)); colorbar();
F = getframe(gcf); imwrite(F.cdata,'svm_test_3d.png','png');
view([0,0,1]); daspect([1,1,1]);axis([-15,15,-15,15])
F = getframe(gcf); imwrite(F.cdata,'svm_test_proj.png','png');

% se2
% svm = load('test_se2.txt');
% h = scatter3(svm(:,1),svm(:,2),svm(:,3),4,svm(:,4)); colorbar();
% F = getframe(gcf); imwrite(F.cdata,'se2_3d.png','png');
% view([0,0,1]); daspect([1,1,1]);
% F = getframe(gcf); imwrite(F.cdata,'se2_proj.png','png');