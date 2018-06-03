import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import plotly as py
import plotly.graph_objs as go

import glob


N_POP = 4

def plotRewards():
    # log = pd.read_csv("/home/ben/Documents/exploration_data/NE/logs2018-06-02-14-21-36/rewards.csv")
    log = pd.read_csv("/home/ben/Documents/exploration_data/PG/logs2018-06-02 00:24:04/rewards.csv")
    # log = pd.read_csv("/home/ben/Documents/exploration_data/logs2018-05-31-22-56-15/rewards.csv", header=0)
    #get_num = lambda x: x.split(' ')[1]
    #rewards = log.iloc[:, 0].apply(get_num)

    # rewards = log['episode_reward']
    rewards = log.iloc[:, 0]
    plt.plot(range(len(rewards)), pd.to_numeric(rewards))

    pop_delimiters = np.array([i for i in range(0,len(rewards),N_POP)])
    pop_means = np.zeros(len(rewards))
    pop_medians = np.zeros(len(rewards))
    for i, l in enumerate(pop_delimiters):
        plt.axvline(l, color='r')
        pop_means[i] = np.mean(rewards[l:min(l+N_POP, len(rewards))])
        pop_medians[i] = np.median(rewards[l:min(l+N_POP, len(rewards))])
        #print(pop_means)
        
    plt.hlines(pop_means, pop_delimiters, pop_delimiters + N_POP)
    plt.hlines(pop_medians, pop_delimiters, pop_delimiters + N_POP, color='g')

    plt.xlabel("Episode")
    plt.ylabel("Reward (explored area in m^2")
    plt.title("Neural Evolution with episode length = 5m")
    plt.show()


def plotExploredArea():
    files = glob.glob("/home/ben/Documents/exploration_data/NE/logs2018-06-02-14-21-36/explored_area*")
    # files = glob.glob("/home/ben/Documents/exploration_data/PG/logs2018-06-02 00:24:04/explored_area*")
    traces = []
    for file in files:
        log = pd.read_csv(file, header=0)
        #print(log.iloc[2:,:])
        #log = pd.to_numeric(log.iloc[2:,:], errors='ignore')
        for col in range(1, log.shape[1]):
            plt.plot(log['time'], pd.to_numeric(log.iloc[:, col], errors='ignore'))
            traces.append(go.Scatter(x = log['time'], y = pd.to_numeric(log.iloc[:, col])))

    plt.title("Explored area")
    plt.xlabel("Time [s]")
    plt.ylabel("Area [m^2]")
    plt.show()

    py.offline.plot(traces[::5])

def main():
    print("Generating plots...")
    # plotExploredArea()
    plotRewards()

if __name__=="__main__":
    main()