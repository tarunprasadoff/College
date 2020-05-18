st=0;
os=25;
obs=[st,st;st+os,st;st+2*os,st;st+3*os,st;st,st+os;st+os,st+os;st+2*os,st+os;st+3*os,st+os;st,st+2*os;st+os,st+2*os;st+2*os,st+2*os;st+3*os,st+2*os;st,st+3*os;st+os,st+3*os;st+2*os,st+3*os;st+3*os,st+3*os];
obs=-2.5*ones(16,2)+obs;
plot(obs(:,1),obs(:,2), 's', 'MarkerSize',10);
axis([-50 150 -50 150]);