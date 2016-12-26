load R_Torques.dat
Torques = R_Torques
subplot(3,2,1), plot(Torques(:,1),Torques(:,2))
subplot(3,2,2), plot(Torques(:,1),Torques(:,5))
subplot(3,2,3), plot(Torques(:,1),Torques(:,3))
subplot(3,2,4), plot(Torques(:,1),Torques(:,6))
subplot(3,2,5), plot(Torques(:,1),Torques(:,4))
subplot(3,2,6), plot(Torques(:,1),Torques(:,7))
