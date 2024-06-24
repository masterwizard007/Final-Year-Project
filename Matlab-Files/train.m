% Load the Simulink environment
env = rlSimulinkEnv('rl_learn2/Environment', 'rl_learn2/Agent');

% Define the DQN agent
agent = rlDQNAgent(env.ObservationInfo, env.ActionInfo);

% Set DQN agent options
opt = rlDQNAgentOptions;
opt.TargetSmoothFactor = 1e-3;
opt.ExperienceBufferLength = 1e6;
opt.TargetUpdateFrequency = 4;
opt.DiscountFactor = 0.99;

% Define training parameters
episodes = 100;
stepsPerEpisode = 100;

% Train the agent
trainOpts = rlTrainingOptions;
trainOpts.MaxEpisodes = episodes;
trainOpts.MaxStepsPerEpisode = stepsPerEpisode;
trainOpts.ScoreAveragingWindowLength = 50;
trainOpts.StopTrainingCriteria = 'AverageReward';
trainOpts.StopTrainingValue = 100;

% Train the agent
trainingStats = train(agent, env, trainOpts);

% Plot training statistics
plot(trainingStats.EpisodeReward)
xlabel('Episode')
ylabel('Total Reward')
title('Training Progress')


