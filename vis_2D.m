
%% Less test data
Nte = 1e4; Ndte = floor(nthroot(Nte,E));  Nte = Ndte^E;
Xtes = ndgridj(XteMin, XteMax,Ndte*ones(E,1)) ;
Xtes1 = reshape(Xtes(1,:),Ndte,Ndte); Xtes2 = reshape(Xtes(2,:),Ndte,Ndte);


%% Plot full trajectories
figure;  hold on; xlabel('x_1'); ylabel('x_2');
plot(Xsim(1,:),Xsim(2,:),'k');
plot(Xd(1,:),Xd(2,:),'g--');
plot(Xtr(1,:),Xtr(2,:),'kx');
surf(Xtes1,Xtes2,reshape(sigfun(Xtes).^2-1e5,Ndte,Ndte),'EdgeColor','none','FaceColor','interp'); colormap(flipud(parula));
legend('x(t)','x_d(t)', 'x_i','\sigma(x)'); % view([20 20]);


%% Plot tracking error
norme = sqrt(sum((Xsim-Xd).^2,1));
figure;  xlabel('t'); ylabel('|e| and |B|');
semilogy(T,norme);
% figure; xlabel('t');
hold on; 
semilogy(T,AreaError);
legend('|e|','|B|');

%% Plot f vs fh
figure; hold on; xlabel('x_1'); ylabel('x_2');  title('f(x) vs \nu(x)');
surf(Xtes1,Xtes2,reshape(pFeLi.f(Xtes),Ndte,Ndte),'edgecolor','none');
plot3(Xtes(1,:),Xtes(2,:),pdyn.f(Xtes),'b*');
plot3(Xtr(1,:),Xtr(2,:),Ytr,'kx');
legend('\nu(x)','f(x)', 'y_i');  view([20 20]);

%% Animation
figure;  hold on; xlim([XteMin(1) XteMax(2)]);ylim([XteMin(2) XteMax(2)]);
xlabel('x_1'); ylabel('x_2');

for it=1:length(T)
            Xhull = Xte(:,ihull{it});
    if it>1,        delete(timestamp); delete(h); end
    if it ==length(T)
                h = fill(Xhull(1,:),Xhull(2,:),'r');
        plot(Xsim(1,1:it),Xsim(2,1:it),'k');
        plot(Xd(1,1:it),Xd(2,1:it),'g--');
        plot(Xtr(1,:),Xtr(2,:),'kx');
    else
        h = fill(Xhull(1,:),Xhull(2,:),'r','HandleVisibility','off');
        plot(Xsim(1,1:it),Xsim(2,1:it),'k','HandleVisibility','off');
        plot(Xd(1,1:it),Xd(2,1:it),'g--','HandleVisibility','off');
        plot(Xtr(1,:),Xtr(2,:),'kx','HandleVisibility','off');
    end
    timestamp = text(XteMin(1)+0.5,XteMin(2)+0.5,['t=' num2str(T(it))]);
%     htext =  text(Xd(1,it),Xd(2,it)-1,num2str(AreaError(it)));


    drawnow;
    pause(0.02);
end
legend('B','x(t)','x_d(t)','x_i');