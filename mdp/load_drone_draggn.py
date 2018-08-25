import torch
from i_draggn import ProgArgNet
from i_draggn import variable_wrap
import numpy as np
import re


def parse_input(cmd, word2id):
    """
    - Parses the input.
    - Translates to the ID version.
    - Wraps it into a tensor + varibale.

    :param cmd: string, the input NL command
    :param word2id: a dictionary that maps from a word to its ID
    :return:
    """
    cmd = cmd.lower()
    cmd = cmd.replace(".", "")
    cmd = cmd.replace("!", "")
    cmd = cmd.replace("?", "")
    cmd = cmd.replace(",", " ,")
    cmd_array = cmd.split(" ")
    encoded_cmd = np.zeros((1, 30))
    for i in range(len(cmd_array)):
        if cmd_array[i] in word2id:
            encoded_cmd[0][i] = word2id[cmd_array[i]]
    return variable_wrap(torch.LongTensor(encoded_cmd))


def run(command):
    main_input = command

    # 1) Loading models:
    arg_model = torch.load("arg_model_saved.pt")
    prog_model = torch.load("prog_model_saved.pt")

    # 2) Loading dictionaries and creating inverse:
    word2id = np.load('word2id.npy').item()
    prog2id = np.load('prog2id.npy').item()
    arg2id = np.load('arg2id.npy').item()

    id2prog = dict([value, key] for key, value in prog2id.items())
    assert len(id2prog) == len(prog2id), "There is a duplicate value in prog2id."
    id2arg = dict([value, key] for key, value in arg2id.items())
    assert len(id2arg) == len(arg2id), "There is a duplicate value in arg2id."

    # 3) Testing:
    test1 = parse_input(main_input, word2id)
    test1_prog = (prog_model(test1).data.max(1, keepdim=True)[1]).numpy()[0][0]
    test1_arg = (arg_model(test1).data.max(1, keepdim=True)[1]).numpy()[0][0]

    print("Input sentence is: ", main_input)
    print("Predicted prog is: ", id2prog[test1_prog])
    print("Predicted arg is: ", id2arg[test1_arg])
    return str(id2prog[test1_prog]), str(id2arg[test1_arg])


if __name__ == '__main__':
    main_input = "go to blue room"

    # 1) Loading models:
    arg_model = torch.load("arg_model_saved.pt")
    prog_model = torch.load("prog_model_saved.pt")

    # 2) Loading dictionaries and creating inverse:
    word2id = np.load('word2id.npy').item()
    prog2id = np.load('prog2id.npy').item()
    arg2id = np.load('arg2id.npy').item()

    id2prog = dict([value, key] for key, value in prog2id.items())
    assert len(id2prog) == len(prog2id), "There is a duplicate value in prog2id."
    id2arg = dict([value, key] for key, value in arg2id.items())
    assert len(id2arg) == len(arg2id), "There is a duplicate value in arg2id."

    # 3) Testing:
    test1 = parse_input(main_input, word2id)
    test1_prog = (prog_model(test1).data.max(1, keepdim=True)[1]).numpy()[0][0]
    test1_arg = (arg_model(test1).data.max(1, keepdim=True)[1]).numpy()[0][0]

    print("Input sentence is: ", main_input)
    print("Predicted prog is: ", id2prog[test1_prog])
    print("Predicted arg is: ", id2arg[test1_arg])