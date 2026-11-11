function detection = apply_term_cond(m)
% Coverage termination of the funnel sampling: stop once m consecutive
% samples fell into existing funnels, with m large enough that the free
% space is covered to a fraction alpha with confidence Pc.
alpha = 0.95;
Pc = 0.95;
detection = m >= (log(1-Pc)/log(alpha))-1;
end
