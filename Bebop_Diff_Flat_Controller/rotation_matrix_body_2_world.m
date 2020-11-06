function R = rotation_matrix_body_2_world(phi, theta, psi)
% Z-X-Y rotation matrix

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];

Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];

Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

R = Rz*Rx*Ry;

end