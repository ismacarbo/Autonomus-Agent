time = 1;
v0val = 10;
a0val = 0;
xfval = 90;
vfval = 2.;
afval = 2.;

%% Test Pass Primitive
[space] = s_opt(time, v0val, a0val, xfval, vfval, afval, Tmax);
[vel] = v_opt(time, v0val, a0val, xfval, vfval, afval, Tmax);
[acc] = a_opt(time, v0val, a0val, xfval, vfval, afval, Tmax);
[jerk] = j_opt(time, v0val, a0val, xfval, vfval, afval, Tmax);
[cost] = total_cost(v0val, a0val, xfval, vfval, afval, Tmax);
[coef] = coef_list(v0val, a0val, xfval, vfval, afval, Tmax);