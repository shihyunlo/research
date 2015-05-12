%toy problem for piece-wise update law
b = 2;
a = 2;
d = 0.5;
toyDynamics = @(x,u)-a*x+b*(u+d);
toyDynamics2 = @(x)-Am*x;

Am = 10;
dt = 0.005;
T = 5;
state = 0;
S = [];
SIG = [];
S_p = [];
s_d = 0;
sig = 0;
state_p = 0;
PHI = 1/Am*(1-exp(-Am*dt));
alpha = 0.90;
for t = 0:dt:T

    [~,state_new] = ode45(@(t, state)toyDynamics(state, s_d) , [0 dt], state); 
    state = state_new(end,:)';
    
    state_p_dot = -Am*state+b*(s_d+sig);
    state_p = state_p + state_p_dot*dt;
    S_p = [S_p state_p];
    
    mu = exp(-Am*dt)*(state_p-state);
    sig = -1/b/PHI*mu;
    s_d = alpha*s_d -(1-alpha)*sig;
    S = [S state];
    SIG = [SIG sig];
    
end
    
    plot(SIG)
    figure
    plot(0:dt:T,S_p,0:dt:T,S,'r')
  
