import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def plotNE():
    log = pd.read_csv("~/.ros/rewards.csv", header=0)
    #get_num = lambda x: x.split(' ')[1]
    #rewards = log.iloc[:, 0].apply(get_num)

    
    print(log)

    rewards = log['episode_reward']
    print(rewards)
    plt.plot(range(len(rewards)), pd.to_numeric(rewards))
    plt.show()

def plotExploredArea():
    log = pd.read_csv("~/.ros/explored_area.csv", header=0)
    #get_num = lambda x: x.split(' ')[1]
    #rewards = log.iloc[:, 0].apply(get_num)
    log = pd.to_numeric(log, errors='ignore')
    print(log[230:250])
    print(log.applymap(lambda x: isinstance(x, (str)))[230:250])
    # print(log.applymap(np.isreal)[230:250])

    for col in range(log.shape[1]):
     plt.plot(log['time'], pd.to_numeric(log.iloc[:, col + 1], errors='ignore'))
    
    plt.show()

def main():
    print("Generating plots...")
    plotExploredArea()

if __name__=="__main__":
    main()