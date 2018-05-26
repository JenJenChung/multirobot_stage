import matplotlib.pyplot as plt
import pandas as pd

def main():
    print("Generating plots...")

    log = pd.read_csv("~/.ros/rewards.txt", header=0)
    #get_num = lambda x: x.split(' ')[1]
    #rewards = log.iloc[:, 0].apply(get_num)

    
    print(log)

    rewards = log['episode_reward']
    print(rewards)
    plt.plot(range(len(rewards)), pd.to_numeric(rewards))
    plt.show()

if __name__=="__main__":
    main()