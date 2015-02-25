function q = normalize_angles(q,qL,qU)
%
% utility function used in example_IK_ABB.m
%
% q  - angles [rad.]
% qL - lower limit for joint angles [rad.]
% qU - upper limit for joint angles [rad.]
%

for i=1:length(q)
    while q(i)>qU(i)
        q(i)= q(i)-2*pi;
    end
    while q(i)<qL(i)
        q(i)= q(i)+2*pi;
    end
end

%%%EOF