from __future__ import print_function
from torch.utils.data import Dataset

P_IDX, A_IDX = 0, 1
max_sentence_len = 50


class DraggnDataset(Dataset):
    """
    A Dataset class that sorts data for the DRAGGN project. Helps with shuffling and batching.

    """
    def __init__(self, x_data, y_data):
        self.sentences = x_data
        self.progs = y_data[:, P_IDX]
        self.args = y_data[:, A_IDX]

    def __getitem__(self, index):
        return self.sentences[index], self.progs[index], self.args[index]

    def __len__(self):
        return len(self.sentences)


if __name__ == "__main__":
    """
    A main function to test the DraggnDataset dataset.

    """
    # x_data: numpy matrix of rows of words
    # y_data: numpy matrix where each row row[0] is prog and row[1] is arg
    # #         (1st column is programs and 2nd column is arguments)
    #
    # train_dataset = DraggnDataset()
    # train_loader = DataLoader(dataset=train_dataset, batch_size=10, shuffle=True)


