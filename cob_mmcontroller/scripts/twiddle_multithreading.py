#! /usr/bin/env python

import os
import sys
import glob
import time as t
import multiprocessing
import matplotlib.pyplot as plt
from ast import literal_eval #to convert string to dict
from copy import deepcopy

params = {'Kp': [1.0, 0.1], 'Ki': [1.0, 0.1], 'Kd': [1.0, 0.1], 'T1': [1.0, 0.1], 'T2': [1.0, 0.1]}
model_dict = {#'P': "params['Kp'][0]*u", 
              #y = Kp*u
              #'PT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + T1/dt*y_m1)
              'I': "params['Ki'][0]*u_sum", 
              'IT1': "1/(params['T1'][0]/dt + 1) *(params['Ki'][0]*u_sum + params['T1'][0]/dt*y_m1)", 
              'IT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Ki'][0]*u_sum + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              #####'IT2': "params['Ki'][0]*u_sum - 2*params['T1'][0]/dt*params['T2'][0]/dt*y_m1 - (params['T2'][0]*params['T2'][0])/(dt*dt)",
              ##y = Ki*u_sum
              'PI': "params['Kp'][0]*u + params['Ki'][0]*u_sum", 
              ##y = Kp*u + Ki*u_sum
              'PIT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + T1/dt*y_m1)
              'PIT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              #####'PIT2': "params['Kp'][0]*u + params['Ki'][0]*u_sum - 2*params['T1'][0]/dt*params['T2'][0]/dt*y_m1 - (params['T2'][0]*params['T2'][0])/(dt*dt)",
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              #'PD': "params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Kd*(u - u_m1)/dt
              #'PDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              #'PDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)", 
              ##y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              #'PID': "params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt", 
              ##y = Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt
              #'PIDT1': "1/(params['T1'][0]/dt + 1) *(params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + params['T1'][0]/dt*y_m1)", 
              ##y = 1/(T1/dt + 1) *(Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + T1/dt*y_m1)
              #'PIDT2': "1/(params['T2'][0]/(dt*dt) + params['T1'][0]/dt + 1) * (params['Kp'][0]*u + params['Ki'][0]*u_sum + params['Kd'][0]*(u - u_m1)/dt + (2*params['T2'][0]/(dt*dt) + params['T1'][0]/dt)*y_m1 - params['T2'][0]/(dt*dt)*y_m2)" 
              #y = 1/(T2/(dt*dt) + T1/dt + 1) * (Kp*u + Ki*u_sum + Kd*(u - u_m1)/dt + (2*T2/(dt*dt) + T1/dt)*y_m1 - T2/(dt*dt)*y_m2)
              }


def main():
    ran_counter = 0
    processID = 1
    process_list = []
    path = sys.argv[1]
    data_file_dict = {}

    print "%d files found"%len(glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "*", "data_set")))
    for data_file in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*", "*", "data_set")):
        data_file_abspath = os.path.abspath(data_file)
        with open(data_file_abspath, 'r') as f_in:
            data_set = literal_eval(f_in.read())

        interesting_data_list = []
        for name, data in data_set.iteritems():
            if name != 'time':
                if abs(sum(data[2])) > 0.001:
                    interesting_data_list.append(name)

        data_file_dict[data_file_abspath] = interesting_data_list


    for data_file_path, data_list in data_file_dict.iteritems():
        with open(data_file_path, 'r') as f_in:
            data_set = literal_eval(f_in.read())
        time = data_set['time']
        for data_name in data_list:
            data = data_set[data_name]
            for model_name, model in model_dict.iteritems():
                while len(process_list) > multiprocessing.cpu_count()-1:
                    t.sleep(0.01)
                    for num, pro in enumerate(process_list):
                        if not pro.is_alive():
                            process_list.pop(num)
                            ran_counter += 1
                            print "%d processes left to finish"%(len(data_file_dict)*len(model_dict) - ran_counter)
                #print len(process_list)

                print "%d of %d processes were started"%(processID, len(data_file_dict)*len(model_dict))
                p = fittingProcess(processID, "Process-"+str(processID), os.path.dirname(data_file_path), data, data_name, model, model_name, time)
                p.start()
                processID += 1
                process_list.append(p)

    while not process_list == []:
        t.sleep(0.01)
        for num, pro in enumerate(process_list):
            if not pro.is_alive():
                process_list.pop(num)
                #ran_counter += 1
                print len(process_list), "processes left to finish"

    print "All files processed"

    for direction_dir in glob.glob(os.path.join(sys.path[0], "..", "data", path, "*")):
        direction_dir_abs = os.path.abspath(direction_dir)
        parameter_dict_all = {}

        if os.path.isdir(direction_dir_abs):
            for twist_dir in glob.glob(os.path.join(direction_dir_abs, "*")):
                twist_dir_abs = os.path.abspath(twist_dir)
                if os.path.isdir(twist_dir_abs):
                    result_list = []
                    for result_file in glob.glob(os.path.join(twist_dir_abs, "result_*")):
                        with open(result_file, 'r') as f_res_in:
                            result_list.append(literal_eval(f_res_in.read()))

                    # get best model
                    error_list = []
                    for result in result_list:
                        error_list.append(result[3])
                    best_result_index = error_list.index(min(error_list))
                    with open(os.path.join(twist_dir_abs, "best_model_%s"%result_list[best_result_index][2]), 'w') as f_best_res:
                        f_best_res.write(str(result_list[best_result_index]))

                    # add parameter
                    for result in result_list:
                        if result[2] not in parameter_dict_all:
                            parameter_dict_all[result[2]] = []
                        parameter_dict_all[result[2]].append(result[4])

        plot_dict = {}
        plot_color_list = ['r', 'b', 'g', 'm', 'c', 'y', 'k', 'r', 'b', 'g', 'k', 'y', 'c', 'm', '', '']
        plot_line_conf = ['o', 's', '^', '*', '+', 'v', 'x', 'd', 'p', 'o', 's', '^', '*', '+', '', '', '', '']
        model_index = 0
        model_name_list = []
        param_name_list = ['Kd', 'Ki', 'T2', 'T1', 'Kp', 'T']#['Kd', 'Ki', 'Kp', 'T2', 'T1']
        param_name_abbr_list = ['D', 'I', '2', 'T', 'P', '']#['D', 'I', 'P', '2', 'T']
        for model_name, parameter_dict_list in parameter_dict_all.iteritems():
            model_index += 1
            twist_index = 0
            model_name_list.append(model_name)
            print model_name_list
            for parameter_dict in parameter_dict_list:
                twist_index += 1
                param_index = 0
                for param_name, value_list in parameter_dict.iteritems():
                    print param_name
                    param_index += 1
                    if param_name_abbr_list[param_index-1] in model_name:    
                        plt.figure(param_index)
                        if twist_index == 1:
                            plt.plot(twist_index+0.05*model_index, value_list[0], plot_color_list[model_index-1]+'*', label=model_name)
                        else:
                            plt.plot(twist_index+0.05*model_index, value_list[0], plot_color_list[model_index-1]+'*')

        for i in range(param_index):
            plt.figure(i+1)
            plt.title(param_name_list[i])
            plt.legend(bbox_to_anchor=(.5, -.12), loc=8, ncol=4, borderaxespad=0.)
            plt.savefig(os.path.join(direction_dir_abs, "params_"+str(i+1)+".pdf"), format='pdf')
            plt.clf()    



    
class fittingProcess(multiprocessing.Process):
    def __init__(self, processID, processName, dir_path, data, data_name, model, model_name, time):
        self.processID = processID
        self.processName = processName
        self.dir_path = dir_path
        self.data = data
        self.data_name = data_name
        self.model = model
        self.model_name = model_name
        self.time = time
        print "%s: started"%processName
        multiprocessing.Process.__init__(self)

    def run(self):
        print "%s: fitting model %s for %s of twist %s"%(self.processName, self.model_name, self.data_name, os.path.basename(self.dir_path))
        (best_error, params) = self.twiddle_delay(self.model, self.data, self.time)
        plt.figure(self.processID)
        plt.plot(self.time, self.data[0], '--')
        model_output = self.run_model(params, self.model, self.data, self.time)
        plt.plot(self.time[:len(model_output[1])], model_output[1])
        plt.suptitle(os.path.basename(self.dir_path)+" "+self.model_name, fontsize=10)
        plt.title(" ".join([(k + ': ' + "%.3f" %v[0]) for k,v in params.iteritems()]), fontsize=8)
        plt.savefig(os.path.join(self.dir_path, self.data_name+"_"+self.model_name+".pdf"), format="pdf")
        #plt.clf()
        print "%s: %s plot saved into pdf"%(self.processName, self.model_name)

        with open(os.path.join(self.dir_path, "result_%s_%s"%(self.data_name, self.model_name)), 'w') as f_res:
            f_res.write(str([os.path.basename(self.dir_path), self.data_name, self.model_name, best_error, params, model_output[1]]))

        print "%s: finshed"%self.processName
        return



    def twiddle_delay(self, model, data, time, tol = 0):
        params = {'Kp': [1.0, 0.1], 'Ki': [1.0, 0.1], 'Kd': [1.0, 0.1], 'T1': [1.0, 0.1], 'T2': [1.0, 0.1], 'T': [10, 2]}
        change = True

        (best_error, params) = self.twiddle_params(model, data, time, params)
        while change or params['T'][1] != 0:
            delay_temp = params['T'][0]
            change = True
            params['T'][0] += params['T'][1]
            #print self.processName, params['T']
            (err, new_params) = self.twiddle_params(model, data, time, self.reset_param_deltas(params))
            if err < best_error:
                best_error = err
                #params = new_params
                params['T'][1] += 1
                print self.processName, "+"#, params['T']
            else:
                if params['T'][0] - 2*params['T'][1] >= 0: params['T'][0] -= 2*params['T'][1]
                else: params['T'][0] = 0
                #print self.processName, params['T']
                (err, new_params) = self.twiddle_params(model, data, time, self.reset_param_deltas(params))
                if  err < best_error:
                    best_error = err
                    #params = new_params
                    params['T'][1] += 1
                    print self.processName, "-"#, params['T']
                else:
                    params['T'][0] = delay_temp
                    if params['T'][1] > 0: params['T'][1] -= 1#; print self.processName, "0"
                    change = False
                    print self.processName, "0"
            print self.processName, params['T']

        return best_error, params

                
            
            
    def twiddle_params(self, model, data, time, input_params=params, tol = 0.0001):
        # deepcopy of input_params
        params = deepcopy(input_params)

        n = 0
        best_error = self.run_model(params, model, data, time)[0]
        while sum([x[1] for k,x in params.iteritems() if k != 'T']) > tol: 
            #if n > 10e9 and sum([x[1] for x in params.itervalues()[:-1]]) < 10*tol:
            #    print "Too many steps"
            #    break
            for name in params.iterkeys():
                if name == 'T':
                    continue

                params[name][0] += params[name][1]
                err = self.run_model(params, model, data, time)[0]
                if err < best_error:
                    best_error = err
                    params[name][1] *= 1.1
                else:
                    params[name][0] -= 2*params[name][1]
                    err = self.run_model(params, model, data, time)[0]
                    if  err < best_error:
                        best_error = err
                        params[name][1] *= 1.1
                    else:
                        params[name][0] += params[name][1]
                        params[name][1] *= 0.9
                #print "\r", sum([x[1] for x in params.itervalues()]), best_error, [x[0] for x in params.itervalues()], n, "\t\t\t",
            n += 1
        return best_error, params



                    
    def run_model(self, params, model, data, time):
        twist_start_index = -1
        y_list = []

        error = 0.0 
        error_sum = 0.0
        
        u_sum = 0.0
        u_m1 = 0.0

        y_m1 = 0.0
        y_m2 = data[0][0]

        dt_m1 = 0.0

        for i in range(len(data[0])):

            dt = time[i] - dt_m1
            if dt == 0.0: continue

            if data[2][i] == 0. and twist_start_index < 0:
                u = 0.
                y = 0.
            else:
                if twist_start_index < 0: twist_start_index = i
                if i < twist_start_index+params['T'][0]:
                    u = 0.
                else:
                    u = data[2][i-params['T'][0]]
            u_sum += u
            y = eval(model)
            y_list.append(y)
            error = y - data[0][i]
            
            y_m2 = y_m1
            y_m1 = y
            u_m1 = u
            dt_m = time[i]

            error_sum += abs(error)

        return error_sum, y_list


    def reset_param_deltas(self, params):
        new_params = {}
        for key, value in params.iteritems():
            if key != 'T':
                new_params[key] = [value[0], value[1]+0.01]
            else:
                new_params[key] = value
        
        return new_params



if __name__ == '__main__':
    main()
