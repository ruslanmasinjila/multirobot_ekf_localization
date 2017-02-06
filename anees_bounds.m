%   computes the lower and upper anees bounds

function [lower_bound,upper_bound] = anees_bounds(numRuns)

n=3;
lower_bound=(1-(2)/(9*n*numRuns)-1.96*sqrt((2)/(9*n*numRuns)))^3;
upper_bound=(1-(2)/(9*n*numRuns)+1.96*sqrt((2)/(9*n*numRuns)))^3;