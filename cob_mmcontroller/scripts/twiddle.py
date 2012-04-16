#! /usr/bin/env python

import os
import sys
from ast import literal_eval #to convert string to dict

model_dict = {'P': "params['Kp']*u", 
              #y = Kp*u
              'PT1': "1/(params['T1']/dt + 1) *(params['Kp']*u + params['T1']/dt*y_m1)", 
              #y = 1/(T1/dt + 1) *(Kp*u + T1/dt*y_m1)
              'I': "params['Ki']*u_sum", 
              #y = Ki*u_sum
              'PI': "params['Kp']*u + params['Ki']*u_sum", 
              #y = Kp*u + Ki*u_sum
              'PIT1': "1/(params['T1']/dt + 1) *(params['Kp']*u + params['Ki']*u_sum + params['T1']/dt*y_m1)", 
              #y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + T1/dt*y_m1)
              'PIT2': "1/(params['T2']/(dt*dt) + params['T1']/dt + 1) * (params['Kp']*u + params['Ki']*u_sum + (2*params['T2']/(dt*dt) + params['T1']/dt)*y_m1 - params['T2']/(dt*dt)*y_m2)", 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              'PD': "params['Kp']*u + params['Kd']*(u - u_m1)/dt", 
              #y = Kp*u + Kd*(u - u_m1)/dt
              'PDT1': "1/(params['T1']/dt + 1) *(params['Kp']*u + params['Kd']*(u - u_m1)/dt + params['T1']/dt*y_m1)", 
              #y = 1/(T1/dt + 1) *(Kp*u + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              'PDT2': "1/(params['T2']/(dt*dt) + params['T1']/dt + 1) * (params['Kp']*u + params['Kd']*(u - u_m1)/dt + (2*params['T2']/(dt*dt) + params['T1']/dt)*y_m1 - params['T2']/(dt*dt)*y_m2)", 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              'PID': "params['Kp']*u + params['Ki']*u_sum + params['Kd']*(u - u_m1)/dt", 
              #y = Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt
              'PIDT1': "1/(params['T1']/dt + 1) *(params['Kp']*u + params['Ki']*u_sum + params['Kd']*(u - u_m1)/dt + params['T1']/dt*y_m1)", 
              #y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              'PIDT2': "1/(params['T2']/(dt*dt) + params['T1']/dt + 1) * (params['Kp']*u + params['Ki']*u_sum + params['Kd']*(u - u_m1)/dt + (2*params['T2']/(dt*dt) + params['T1']/dt)*y_m1 - params['T2']/(dt*dt)*y_m2)" 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              }


def main():
    path = sys.argv[1]


    for data_file in glob.iglob(os.path.join("..", "data", path, "*", "data_set"):
        print data_file

        with open(data_file, 'r') as f_in:
            data_set = literal_eval(f_in.read())
        
        result = parse_data_set(model, data_set)
        with open(os.path.join(os.path.dirname(data_file), result), 'w') as f_out:
            f_out.write(str(result))



def parse_data_set(model, data_set):
    time = data_set['time']
    twist = {}
    for name, data in data_set.iteritems():
        print name
        if name != 'time':
            if sum(data[2]) > 0.001:
                twist[name] = []
                for model_name, model in model_dict.iteritems():
                    (best_error, params) = twiddle(model, data, time)
                    twist[name].append([best_error, model_name, params])
                twist[name].sort()
        print best_error, model_name, params

    return twist

            
        
        
def twiddle(model, data, time, tol = 0.001):
    params = {'Kp': [0.0, 1.0], 'Ki': [0.0, 1.0], 'Kd': [0.0, 1.0], 'T1': [0.0, 1.0], 'T2': [0.0, 1.0]}

    best_error = run_model(params, model, data, time)
    while sum([x[1] for x in params.itervalues()]) > tol: 
        for name in params.iterkeys():
            params[name][0] += params[name][1]
            err = run_model(params, model, data, time)
            if err < best_error:
                best_error = err
                params[name][1] *= 1.1
            else:
                params[name][0] -= 2*params[name][1]
                err = run_model(params, model, data, time)
                if  err < best_error:
                    best_errror = err
                    params[name][1] *= 1.1
                else:
                    params[name][0] += params[name][1]
                    params[name][1] *= 0.9
            print "\r", sum([x[1] for x in params.itervalues()]), best_err,

    return best_error, params



                
def run_model(params, model, data, time):
    error = 0.0
    error_sum

    u_sum = 0.0
    u_m1 = 0.0

    y_m1, y_m2 = data[0][0]

    dt_m1 = 0.0

    for i in range(len(data)):
        u = data[2][i]
        u_sum += u

        dt = time[i] - dt_m1

        y = eval(model)
        error = y - data[0][i]
        
        y_m2 = y_m1
        y_m1 = y
        u_m1 = u
        dt_m = time[i]

        error_sum += error

    return error_sum


if __name__ == '__main__':
    main()
