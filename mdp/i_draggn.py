from __future__ import print_function
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.autograd import Variable
from draggn_dataset import DraggnDataset
from torch.utils.data import DataLoader
import pandas as pd

# Parsing Variables:
P_IDX, A_IDX = 0, 1
max_sentence_len = 30

# Training Variables:
num_experiments = 2  # overall number of experiments
learning_rate = 0.01  # 0.0001 on DRAGGN paper
embedding_size = 25  # 30 on DRAGGN paper
batch_size = 16  # 16 on DRAGGN paper
drop_prob = 0.1  # 0.5 on DRAGGN paper
num_epochs = 5  # 125 on DRAGGN paper
hidden_size = 32  # ex: 32, 64, 128


def parse(is_unseen):
    """
    - Parses and encodes the data and organizes it into x and y.
    - Returns dictionaries.

    :param is_unseen: bool, specifies whether the tested actions are unseen or not
    :return:
        word2id: dictionary, from words to their ids,
        prog_set: dictionary, from programs to their ids,
        arg_set: dictionary, from arguments to their ids,
        train_x: numpy, training x data,
        train_x_len: numpy, the length of each sentence in the training x data (useful sequences are packed),
        test_actions_x: numpy, testing actions x data,
        test_actions_x_len: numpy, the length of each sentence in the testing actions x data,
        test_goals_x: numpy, testing goals x data,
        test_goals_x_len: numpy, the length of each sentence in the testing goals x data,
        train_y: numpy, training y data (labels),
        test_actions_y: numpy, testing y data for actions,
        test_goals_y: numpy, testing y data for goals
    """
    if is_unseen:
        actions_train_path = "data/pruned/unseen/pruned_unseen_actions_train.csv"
        actions_test_path = "data/pruned/unseen/pruned_unseen_actions_test.csv"
        goals_train_path = "data/pruned/unseen/pruned_goals_train.csv"
        goals_test_path = "data/pruned/unseen/pruned_goals_test.csv"
    else:
        actions_train_path = "data/pruned/vanilla/pruned_actions_train.csv"
        actions_test_path = "data/pruned/vanilla/pruned_actions_test.csv"
        goals_train_path = "data/pruned/vanilla/pruned_goals_train.csv"
        goals_test_path = "data/pruned/vanilla/pruned_goals_test.csv"

    # parsing train data:
    #   1) actions
    actions_train_df = pd.read_csv(actions_train_path, header=None)
    actions_train_cu = actions_train_df[0].tolist()  # callable unit
    actions_train_nl = actions_train_df[1].tolist()  # natural language
    actions_train_cu = list(map(lambda x: x.split(), actions_train_cu))
    actions_train_nl = list(map(lambda x: x.split(), actions_train_nl))

    assert(len(actions_train_cu) == len(actions_train_nl))

    #   2) goals
    goals_train_df = pd.read_csv(goals_train_path, header=None)
    goals_train_cu = goals_train_df[0].tolist()  # callable unit
    goals_train_nl = goals_train_df[1].tolist()  # natural language
    goals_train_cu = list(map(lambda x: x.split(), goals_train_cu))
    goals_train_nl = list(map(lambda x: x.split(), goals_train_nl))

    assert(len(goals_train_cu) == len(goals_train_nl))

    # generating train set:
    train_cu = actions_train_cu + goals_train_cu
    train_nl = actions_train_nl + goals_train_nl
    train_set = list(zip(train_nl, train_cu))

    # parsing test data:
    #   1) actions
    actions_test_df = pd.read_csv(actions_test_path, header=None)
    actions_test_cu = actions_test_df[0].tolist()  # callable unit
    actions_test_nl = actions_test_df[1].tolist()  # natural language
    actions_test_cu = list(map(lambda x: x.split(), actions_test_cu))
    actions_test_nl = list(map(lambda x: x.split(), actions_test_nl))

    assert (len(actions_test_cu) == len(actions_test_nl))

    #   2) goals
    goals_test_df = pd.read_csv(goals_test_path, header=None)
    goals_test_cu = goals_test_df[0].tolist()  # callable unit
    goals_test_nl = goals_test_df[1].tolist()  # natural language
    goals_test_cu = list(map(lambda x: x.split(), goals_test_cu))
    goals_test_nl = list(map(lambda x: x.split(), goals_test_nl))

    assert (len(goals_test_cu) == len(goals_test_nl))

    # generating test set:
    test_actions = list(zip(actions_test_nl, actions_test_cu))
    test_goals = list(zip(goals_test_nl, goals_test_cu))
    test_set = test_actions + test_goals

    # creating sentence vocabulary and getting the maximum sentence length:
    word2id, max_len = {"PAD": 0, "UNK": 1}, 0
    for sent, _ in train_set + test_set:
        if len(sent) > max_len:
            max_len = len(sent)
        for word in sent:
            if word not in word2id:
                word2id[word] = len(word2id)

    print("max_len is before if:", max_len)

    if max_len > max_sentence_len:
        max_len = max_sentence_len

    print("max_len is after if:", max_len)

    # vectorizing train and test inputs:
    train_x, train_x_len = np.zeros((len(train_set), max_len)), np.zeros((len(train_set)), dtype=np.int32)
    test_actions_x, test_goals_x = np.zeros((len(test_actions), max_len)), np.zeros((len(test_goals), max_len))
    test_actions_x_len, test_goals_x_len = np.zeros((len(test_actions)), dtype=np.int32), np.zeros((len(test_goals)),
                                                                                                   dtype=np.int32)

    for i in range(len(train_set)):
        nl_sentence = train_set[i][0]
        train_x_len[i] = min(max_len, len(nl_sentence))
        for j in range(train_x_len[i]):
            train_x[i][j] = word2id[nl_sentence[j]]

    for i in range(len(test_actions)):
        nl_sentence = test_actions[i][0]
        test_actions_x_len[i] = min(max_len, len(nl_sentence))
        for j in range(test_actions_x_len[i]):
            test_actions_x[i][j] = word2id[nl_sentence[j]]

    for i in range(len(test_goals)):
        nl_sentence = test_goals[i][0]
        test_goals_x_len[i] = min(max_len, len(nl_sentence))
        for j in range(test_goals_x_len[i]):
            test_goals_x[i][j] = word2id[nl_sentence[j]]

    # creating prog and arg vocabulary:
    prog_set, arg_set, train_labels, test_actions_labels, test_goals_labels = {}, {}, [], [], []
    for i, program in train_set + test_set:
        if len(program) != 2:
            print(i, program)
        assert (len(program) == 2)
        p, arg = program
        if p not in prog_set:
            prog_set[p] = len(prog_set)
        if arg not in arg_set:
            arg_set[arg] = len(arg_set)

    # separating train and test labels:
    for _, program in train_set:
        prog_key, arg = program
        train_labels.append((prog_set[prog_key], arg_set[arg]))
    assert (len(train_labels) == len(train_x))

    for _, program in test_actions:
        prog_key, arg = program
        test_actions_labels.append((prog_set[prog_key], arg_set[arg]))
    assert (len(test_actions_labels) == len(test_actions_x))

    for _, program in test_goals:
        prog_key, arg = program
        test_goals_labels.append((prog_set[prog_key], arg_set[arg]))
    assert (len(test_goals_labels) == len(test_goals_x))

    # vectorizing train and test labels:
    train_y = np.zeros([len(train_labels), 2])
    test_actions_y, test_goals_y = np.zeros([len(test_actions_labels), 2]), np.zeros([len(test_goals_labels), 2])

    for i in range(len(train_labels)):
        trace = train_labels[i]
        train_y[i][P_IDX] = trace[0]
        train_y[i][A_IDX] = trace[1]

    for i in range(len(test_actions_labels)):
        trace = test_actions_labels[i]
        test_actions_y[i][P_IDX] = trace[0]
        test_actions_y[i][A_IDX] = trace[1]

    for i in range(len(test_goals_labels)):
        trace = test_goals_labels[i]
        test_goals_y[i][P_IDX] = trace[0]
        test_goals_y[i][A_IDX] = trace[1]

    return word2id, prog_set, arg_set, train_x, train_x_len, test_actions_x, test_actions_x_len, test_goals_x, \
           test_goals_x_len, train_y, test_actions_y, test_goals_y


def variable_wrap(tensor):
    """
    Checks if GPU is available and wraps with variable according to that.

    :param tensor: Tensor, the tensor to be wrapped
    :return: Variable, the tensor wrapped in Variable
    """
    if torch.cuda.is_available():
        return Variable(tensor.cuda())
    else:
        return Variable(tensor)


class ProgArgNet(nn.Module):
    """
    A NN model class for programs and arguments.
    """
    def __init__(self, vocab_size, num_classes):
        super(ProgArgNet, self).__init__()
        self.embedding = nn.Embedding(vocab_size, embedding_size)
        self.do = nn.Dropout(p=drop_prob)
        self.gru = nn.GRU(input_size=embedding_size, hidden_size=hidden_size, num_layers=2)
        self.relu = nn.ReLU() # Sigmoid, Relu, SELU (really good)
        # self.fc1 = nn.Linear(hidden_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, num_classes)

    def forward(self, inputs):
        inputs = inputs.t()
        emb = self.embedding(inputs)
        gru_out, hid = self.gru(self.do(emb))
        relu_out = self.relu(hid[-1])
        # fc1_out = self.fc1(self.do(relu_out))
        fc2_out = self.fc2(self.do(relu_out))
        return fc2_out


def train_and_test(is_unseen, experiment_acc, exper):
    """
    Does one experiment of training and testing.

    :param is_unseen: bool, specifies whether the tested actions are unseen or not
    :param experiment_acc: numpy, an array to store the accuracy results from the overall experiments
    :param exper: the number of the current experiment
    :return: Nothing.
    """
    # 1) Printing current setup parameters:
    print("____________________________________")
    print("     CURRENT SETUP PARAMETERS")
    print("____________________________________")
    print("learning_rate is: ", learning_rate)
    print("embedding_size is: ", embedding_size)
    print("batch_size is: ", batch_size)
    print("(if dropout used) drop_prob is: ", drop_prob)
    print("num_epochs is: ", num_epochs)
    print("hidden_size is: ", hidden_size)
    print("is_unseen is: ", is_unseen)
    print("____________________________________")

    # 2) Preparing data:
    print("Preparing data.")
    word2id, prog_set, arg_set, train_x, train_x_len, test_actions_x, test_actions_x_len, test_goals_x, \
        test_goals_x_len, train_y, test_actions_y, test_goals_y = parse(is_unseen)

    train_dataset = DraggnDataset(torch.LongTensor(train_x), torch.LongTensor(train_y))
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True)

    test_actions_dataset = DraggnDataset(torch.LongTensor(test_actions_x), torch.LongTensor(test_actions_y))
    test_actions_loader = DataLoader(dataset=test_actions_dataset, batch_size=batch_size, shuffle=True)

    test_goals_dataset = DraggnDataset(torch.LongTensor(test_goals_x), torch.LongTensor(test_goals_y))
    test_goals_loader = DataLoader(dataset=test_goals_dataset, batch_size=batch_size, shuffle=True)

    # 2) Setting up model, loss and optimizer functions:
    print("Setting up models and other functions.")
    prog_model = ProgArgNet(vocab_size=len(word2id), num_classes=len(prog_set))
    arg_model = ProgArgNet(vocab_size=len(word2id), num_classes=len(arg_set))

    if torch.cuda.device_count() > 1:
        print("Using", torch.cuda.device_count(), "GPUs.")
        prog_model = nn.DataParallel(prog_model)
        arg_model = nn.DataParallel(arg_model)

    if torch.cuda.is_available():
        prog_model.cuda()
        arg_model.cuda()

    criterion = nn.CrossEntropyLoss()
    prog_optimizer = optim.Adam(prog_model.parameters(), lr=learning_rate)
    arg_optimizer = optim.Adam(arg_model.parameters(), lr=learning_rate)

    print("prog_model is: ", prog_model)
    print("arg_model is: ", arg_model)

    # 3) Training both models:
    print("Training both models.")
    prog_model.train()
    arg_model.train()
    for epoch in range(1, num_epochs + 1):

        for i, (sentences, progs, args) in enumerate(train_loader, 1):
            inputs = variable_wrap(sentences)
            prog_labels = variable_wrap(progs)
            arg_labels = variable_wrap(args)

            prog_optimizer.zero_grad()
            arg_optimizer.zero_grad()

            prog_out = prog_model(inputs)
            arg_out = arg_model(inputs)

            prog_loss = criterion(prog_out, prog_labels)
            arg_loss = criterion(arg_out, arg_labels)
            prog_loss.backward()
            arg_loss.backward()
            prog_optimizer.step()
            arg_optimizer.step()

        print("epoch: %d, prog_loss: %1.3f" % (epoch, prog_loss.item()))
        print("epoch: %d, arg_loss: %1.3f" % (epoch, arg_loss.item()))
        print("________________________")
    print("Training finished!")

    # 4) Testing on actions:
    print("Testing actions:")
    num_correct = 0
    test_data_size = len(test_actions_loader.dataset)

    prog_model.eval()
    arg_model.eval()

    for sentences, progs, args in test_actions_loader:
        inputs = variable_wrap(sentences)
        prog_labels = variable_wrap(progs)
        arg_labels = variable_wrap(args)

        prog_out = prog_model(inputs)
        arg_out = arg_model(inputs)

        prog_pred = prog_out.data.max(1, keepdim=True)[1].view_as(prog_labels)
        arg_pred = arg_out.data.max(1, keepdim=True)[1].view_as(arg_labels)

        for i in range(len(sentences)):
            if (prog_pred[i] == prog_labels[i]) and (arg_pred[i] == arg_labels[i]):
                num_correct += 1

    experiment_acc[exper][0] = 100. * num_correct / test_data_size
    print('\nActions test accuracy: {}/{} ({:.0f}%)\n'.format(num_correct, test_data_size, experiment_acc[exper][0]))

    print("Testing actions finished!")

    # 5) Testing on goals:
    print("Testing goals:")
    num_correct = 0
    test_data_size = len(test_goals_loader.dataset)

    for sentences, progs, args in test_goals_loader:
        inputs = variable_wrap(sentences)
        prog_labels = variable_wrap(progs)
        arg_labels = variable_wrap(args)

        prog_model.eval()
        arg_model.eval()

        prog_out = prog_model(inputs)
        arg_out = arg_model(inputs)

        prog_pred = prog_out.data.max(1, keepdim=True)[1].view_as(prog_labels)
        arg_pred = arg_out.data.max(1, keepdim=True)[1].view_as(arg_labels)

        for i in range(len(sentences)):
            if (prog_pred[i] == prog_labels[i]) and (arg_pred[i] == arg_labels[i]):
                num_correct += 1

    experiment_acc[exper][1] = 100. * num_correct / test_data_size
    print('\nGoals test accuracy: {}/{} ({:.0f}%)\n'.format(num_correct, test_data_size, experiment_acc[exper][1]))

    print("Testing goals finished!")


def run_experiment(is_unseen):
    """
    Runs the amount of experiments given and returns their accuracies.

    :param is_unseen: bool, specifies whether the tested actions are unseen or not
    :return: experiment_acc: numpy, an array to store the accuracy results from the overall experiments
    """
    experiment_acc = np.zeros([num_experiments, 2], dtype=np.int32)

    for exper in range(num_experiments):
        train_and_test(is_unseen, experiment_acc, exper)

    return experiment_acc


def print_statistics(experiment_acc):
    """
    A helper method to print statistical details about the experiments.

    :param experiment_acc: numpy, an array to store the accuracy results from the overall experiments
    :return: Nothing.
    """
    print("____________________________________")
    print("The overall statistics are: ")
    for j in range(num_experiments):
        print("Experiment ", j + 1, ": 1) Action Acc = ", experiment_acc[j][0], " 2) Goals Acc = ",
              experiment_acc[j][1])
    print("____________________________________")
    action_acc_mean, goal_acc_mean = experiment_acc.mean(axis=0)
    action_acc_std, goal_acc_std = experiment_acc.std(axis=0)
    print("Average Actions Accuracy is: ", action_acc_mean, "%")
    print("     Standard deviation for Actions is: ", action_acc_std)
    print("Average Goals Accuracy is: ", goal_acc_mean, "%")
    print("     Standard deviation for Goals is: ", goal_acc_std)
    print("____________________________________")


def final_train():
    """
    Runs the final training with all of the data combined.

    :return: Nothing.
    """
    # 1) Preparing data:
    print("Preparing data.")
    word2id, prog_set, arg_set, train_x, train_x_len, test_actions_x, test_actions_x_len, test_goals_x, \
    test_goals_x_len, train_y, test_actions_y, test_goals_y = parse(False)

    np.save('word2id.npy', word2id)
    np.save('prog2id.npy', prog_set)
    np.save('arg2id.npy', arg_set)

    final_train_x = np.concatenate((train_x, test_actions_x, test_goals_x), axis=0)
    # Note: this will not be used
    final_train_y = np.concatenate((train_y, test_actions_y, test_goals_y), axis=0)

    final_train_dataset = DraggnDataset(torch.LongTensor(final_train_x), torch.LongTensor(final_train_y))
    final_train_loader = DataLoader(dataset=final_train_dataset, batch_size=batch_size, shuffle=True)

    # 2) Setting up model, loss and optimizer functions:
    print("Setting up models and other functions.")
    prog_model = ProgArgNet(vocab_size=len(word2id), num_classes=len(prog_set))
    arg_model = ProgArgNet(vocab_size=len(word2id), num_classes=len(arg_set))

    if torch.cuda.device_count() > 1:
        print("Using", torch.cuda.device_count(), "GPUs.")
        prog_model = nn.DataParallel(prog_model)
        arg_model = nn.DataParallel(arg_model)

    if torch.cuda.is_available():
        prog_model.cuda()
        arg_model.cuda()

    criterion = nn.CrossEntropyLoss()
    prog_optimizer = optim.Adam(prog_model.parameters(), lr=learning_rate)
    arg_optimizer = optim.Adam(arg_model.parameters(), lr=learning_rate)

    print("prog_model is: ", prog_model)
    print("arg_model is: ", arg_model)

    # 3) Training both models:
    print("Training both models.")
    prog_model.train()
    arg_model.train()
    for epoch in range(1, num_epochs + 1):

        for i, (sentences, progs, args) in enumerate(final_train_loader, 1):
            inputs = variable_wrap(sentences)
            prog_labels = variable_wrap(progs)
            arg_labels = variable_wrap(args)

            prog_optimizer.zero_grad()
            arg_optimizer.zero_grad()

            prog_out = prog_model(inputs)
            arg_out = arg_model(inputs)

            prog_loss = criterion(prog_out, prog_labels)
            arg_loss = criterion(arg_out, arg_labels)
            prog_loss.backward()
            arg_loss.backward()
            prog_optimizer.step()
            arg_optimizer.step()

        print("epoch: %d, prog_loss: %1.3f" % (epoch, prog_loss.item()))
        print("epoch: %d, arg_loss: %1.3f" % (epoch, arg_loss.item()))
        print("________________________")
    print("Training finished!")
    print("Saving models...")
    torch.save(prog_model, "prog_model_saved.pt")
    torch.save(arg_model, "arg_model_saved.pt")
    print("Saved!")


if __name__ == '__main__':
    vanilla_experiment_acc = run_experiment(False)
    unseen_experiment_acc = run_experiment(True)

    print("____________________________________")
    print("             VANILLA")
    print_statistics(vanilla_experiment_acc)

    print("____________________________________")
    print("             UNSEEN")
    print_statistics(unseen_experiment_acc)

    final_train()
    print("Everything is done.")


