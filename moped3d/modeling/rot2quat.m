function a = rot2quat(R)
% ROT2QUAT - Purpose: rotation to quaternion.
%
% Syntax: a = rot2quat(R)
%
% Input:
%    R: 3x3 rotation.
%
% Output:
%    a: 4x1 quaternion.
%
% fsm a@t robots.ox.ac.uk
  if ~isequal(size(R), [3 3])
    error('R must be 3 x 3')
  end
  
  % By taking certain sums and differences of the elements
  % of R we can obtain all products of pairs a_i a_j with
  % i not equal to j. We then get the squares a_i^2 from
  % the diagonal of R.
  
  a2_a3 = (R(1,2) + R(2,1)) / 4;
  a1_a4 = (R(2,1) - R(1,2)) / 4;
  a1_a3 = (R(1,3) - R(3,1)) / 4;
  a2_a4 = (R(1,3) + R(3,1)) / 4;
  a3_a4 = (R(2,3) + R(3,2)) / 4;
  a1_a2 = (R(3,2) - R(2,3)) / 4;
  
  D = [+1 +1 +1 +1
       +1 +1 -1 -1
       +1 -1 +1 -1
       +1 -1 -1 +1] * 0.25;
  aa = D * [sqrt(sum(R(:).^2) / 3); diag(R)];
  
  % form 4 x 4 outer product a \otimes a:
  a_a = [aa(1) a1_a2 a1_a3 a1_a4
         a1_a2 aa(2) a2_a3 a2_a4
         a1_a3 a2_a3 aa(3) a3_a4
         a1_a4 a2_a4 a3_a4 aa(4)];
  
  % use rank-1 approximation to recover a, up to sign.
  [U, S, V] = svd(a_a);
  a = sqrt(S(1, 1)) * U(:, 1);
end
