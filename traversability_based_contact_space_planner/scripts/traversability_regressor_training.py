import numpy as np
import math
import copy
import random
import time
import pickle
import IPython
import sys
# import matplotlib.pyplot

from sklearn import svm

from structures import *
from config_parameter import *
from transformation_conversion import *
from drawing_functions import *

def main(mode, new_data_filenames):

    ## Load training data
    print('Loading data.')

    try:
        print('Loading goal_index_dict...')
        out_file = open('goal_index_dict','r')
        goal_index_dict = pickle.load(out_file)
        out_file.close()
        print('Done.')
    except Exception:
        print('Not Found.')
        return
    goal_num = len(goal_index_dict)

    try:
        print('Loading traversability_training_data...')
        out_file = open('traversability_training_data_'+mode,'r')
        training_data = pickle.load(out_file)
        out_file.close()
        print('Done. Append newly loaded training data to the training data set.')
    except Exception:
        print('Not Found. Use only newly loaded training data.')
        training_data = []

    new_training_data = []
    for filename in new_data_filenames:
        try:
            print('Loading ' + filename + '...')
            out_file = open(filename,'r')
            tmp_data = pickle.load(out_file)
            new_training_data += tmp_data
            out_file.close()
            print('Done.')
        except Exception:
            print('Not Found. Continuing reading next file.')

    new_data_num = len(new_training_data)

    if new_data_num > 0:

        feature_vectors = [[] for i in range(goal_num)]
        ground_truths = [[] for i in range(goal_num)]
        for data in new_training_data:
            env_transition_feature_vector = data[0] # 5D
            successful_goal_count = data[1]

            for i in range(goal_num):
                feature_vectors[i].append(env_transition_feature_vector[i])
                ground_truths[i].append(successful_goal_count[i] )


        # Filter out some data with zero successful goal counts to balance data.
        new_filtered_training_data = []
        zero_counter = [0] * goal_num
        for i in range(goal_num):

            filtered_feature_vectors = []
            filtered_ground_truths = []

            for j in range(new_data_num):
                if(ground_truths[i][j] > 0 or zero_counter[i] < 500): # limit the number of data of zero successful goal count to balance data
                    filtered_feature_vectors.append(feature_vectors[i][j])
                    filtered_ground_truths.append(ground_truths[i][j])

                    if(ground_truths[i][j] == 0):
                        zero_counter[i] = zero_counter[i]+1

            new_filtered_training_data.append((filtered_feature_vectors,filtered_ground_truths))

        # Append the newly loaded data to the existing training dataset
        training_data += new_filtered_training_data
        print('Storing traversability_training_data...')
        out_file = open('traversability_training_data_'+mode,'w')
        pickle.dump(training_data,out_file)
        out_file.close()
        print('Done.')


    print('Finished loading training data.')

    ## Train the traversability regression models
    traversability_regressor = [None] * goal_num
    for i in range(goal_num):
        random.shuffle(training_data[i])
        print('Goal #: %d, Data Num: %d.'%(i,len(training_data[i][0])))


        X = copy.deepcopy(training_data[i][0])
        y = copy.deepcopy(training_data[i][1])

        max_X = [None] * len(X[0])
        min_X = [None] * len(X[0])

        X_array = np.array(X)

        for j in range(len(max_X)):
            max_X[j] = np.amax(X_array[:,j])
            min_X[j] = np.amin(X_array[:,j])

        max_y = np.amax(y)
        min_y = np.amin(y)

        for j in range(len(X)):
            for k in range(len(X[j])):
                X[j][k] = float(X[j][k]-min_X[k])/float(max_X[k]-min_X[k])
            y[j] = float(y[j]-min_y)/float(max_y-min_y)

        clf = svm.SVR(epsilon=0.01,kernel='rbf',C=500)
        clf.fit(X,y)
        traversability_regressor[i] = [clf,min_X,max_X,min_y,max_y]

    out_file = open('traversability_regressor_'+mode,'w')
    pickle.dump(traversability_regressor,out_file)
    out_file.close()

    IPython.embed()


if __name__ == "__main__":

    mode = ''
    new_data_filenames = [] # add the file which will be loaded to append to training data

    if len(sys.argv) > 1:
        mode = sys.argv[1]

    if mode not in mode_list:
        print('Please specify a mode of the regressor from the list:')
        print(mode_list)
    else:
        main(mode,new_data_filenames)