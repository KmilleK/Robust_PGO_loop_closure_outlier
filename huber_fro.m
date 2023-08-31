function y=huber_fro(x, DIM, varargin)

%HUBER_FRO  Huber penalty function for frobenius norm .
%   For a vector X, HUBER_CIRC(X) computes the Huber penalty function
%
%       HUBER_CIRC(X) =   NORM(X,fro)^2 if NORM(X,fro)<=1,
%                       2*NORM(X,fro)-1 if NORM(X,fro)>=1.
%
%   For matrices only
%
%   HUBER_CIRC(X,DIM), HUBER_CIRC(X,DIM,M), and HUBER_CIRC(X,DIM,M,T) 
%   computes the penalty along the dimension DIM.
%
%   Disciplined convex programming information:
%       HUBER_CIRC is jointly convex in X and T. It is nonomonotonic in X 
%       and nonincreasing in T. Therefore, when used in CVX specifications, 
%       X must be affine and T must be concave (or affine). T must be real.
%       X, on the other hand, may be real or complex.
%
%
%
%

if ~cvx_isaffine( x ),
    error( 'Disciplined convex programming error:\n    HUBER_fro is nonmonotonic in X, so X must be affine.', 1 ); %#ok
end
if nargin < 2, DIM = 'Fro'; end
y = huber_pos( norm( x, DIM ), varargin{:} );