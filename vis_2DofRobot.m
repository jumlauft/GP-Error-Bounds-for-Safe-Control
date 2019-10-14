%% Prepare for plot
Xtaskte = RobotJoint2Task(Xtepos(ipos,:),pdyn);
Xtasktr = RobotJoint2Task(Xtr(ipos,:),pdyn);
[Xsimj2, Xsimj1] = RobotJoint2Task(Xsim(ipos,:),pdyn);
Xjoint1 = [zeros(1,2); Xsimj1(:,frame2plot)'];
Xjoint2 = [Xsimj1(:,frame2plot)'; Xsimj2(:,frame2plot)'];
Xhullte = Xtaskte(:,convhull(Xtaskte','simplify',true));
XtaskIncr = Xtaskte(:,~iiLyapDecr(frame2plot,:));
Xhull = XtaskIncr(:,convhull(XtaskIncr','simplify',true));

%% Plot full trajectories
figure; xlabel('t'); xlabel('x');hold on; title('x vs x_d');
plot(T,Xsim);
plot(T,Xd,'--');
legend('$q_1$','$\dot{q}_1$','$q_2$','$\dot{q}_2$','$q_{1,d}$',...
    '$\dot{q}_{1,d}$','$q_{2,d}$','$\dot{q}_{2,d}$','interpreter','latex');


%% Plot tracking error
norme = sqrt(sum((Xsim-Xd).^2,1));
figure; 
semilogy(T,norme);
xlabel('t'); ylabel('|e|'); title('tracking error');

%% Animation
figure;  hold on; axis equal; %xlim([XteMin(1) XteMax(2)]);ylim([XteMin(2) XteMax(2)]);
xlim([(-2.5) (2.5)]); ylim([(-2) (2)]);

for nsim = 1:Nsim
    cla(gca);
    
    XtaskIncr = Xtaskte(:,~iiLyapDecr(nsim,:));
    
    plot(Xtaskte(1,:),Xtaskte(2,:),'g*');
    plot(Xtasktr(1,:),Xtasktr(2,:),'k+');

    plot(XtaskIncr(1,:),XtaskIncr(2,:),'r*');
    
    rectangle('Position',sR*[-1 -1 2 2],  'Curvature',[1,1],'FaceColor','b');%link segment ends
    rectangle('Position',[Xsimj1(1,nsim)-sR Xsimj1(2,nsim)-sR 2*sR 2*sR],  'Curvature',[1,1],'FaceColor','g');%link segment ends
    
    line([0 Xsimj1(1,nsim)],[0 Xsimj1(2,nsim)]);
    line([Xsimj1(1,nsim) Xsimj2(1,nsim)],[Xsimj1(2,nsim) Xsimj2(2,nsim)]);
    
    drawnow;
    pause(0.05);
end