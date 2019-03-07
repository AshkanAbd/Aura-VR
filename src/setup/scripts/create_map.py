#!/usr/bin/env python3.5

import numpy as np


class creator:
    matrix = None

    def __init__(self):
        self.matrix = None

    def get_matrix(self, matrix: np.ndarray):
        self.matrix = matrix

    def remove_addition(self):
        matrix_1 = self.matrix.T
        matrix_2 = self.operator(self.matrix.copy())
        matrix_3 = np.array(self.operator(matrix_1.copy())).T
        matrix_2[matrix_3 == 100] = matrix_3[matrix_3 == 100]
        return matrix_3

    def operator(self, matrix: np.ndarray):
        b = np.where(matrix == 100)
        b = np.append(b[0], b[1]).reshape(2, len(b[0]))
        c = np.random.randint(100, 101, matrix.shape)
        matrix = matrix[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1]
        c[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1] = matrix
        matrix = c
        for y, x in np.ndindex(b.shape):
            if y == 0:
                continue
            if x == 0:
                matrix[b[0, 0], 0:b[1, 0]] = 100
            if x == b.shape[1] - 1:
                matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
            try:
                if b[0, x] != b[0, x + 1]:
                    matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
                    matrix[b[0, x + 1], 0:b[1, x + 1]] = 100
            except:
                pass
        return matrix


# def operator(matrix: np.ndarray):
#     b = np.where(matrix == 100)
#     b = np.append(b[0], b[1]).reshape(2, len(b[0]))
#     c = np.random.randint(100, 101, matrix.shape)
#     matrix = matrix[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1]
#     c[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1] = matrix
#     matrix = c
#     for y, x in np.ndindex(b.shape):
#         if y == 0:
#             continue
#         if x == 0:
#             matrix[b[0, 0], 0:b[1, 0]] = 100
#         if x == b.shape[1] - 1:
#             matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
#         try:
#             if b[0, x] != b[0, x + 1]:
#                 matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
#                 matrix[b[0, x + 1], 0:b[1, x + 1]] = 100
#         except:
#             pass
#     return matrix
#
#
# a = np.asarray([[-1, -1, -1, -1, -1, -1, -1, -1, -1],
#                 [-1, -1, 100, 100, -1, 100, -1, -1, -1],
#                 [-1, 100, 0, 0, 100, 0, 100, -1, -1],
#                 [-1, 100, 0, 0, 0, 0, 100, -1, -1],
#                 [-1, -1, 100, 100, -1, 100, -1, -1, -1],
#                 [-1, -1, -1, 100, -1, 100, -1, -1, -1],
#                 [-1, -1, -1, 100, -1, 100, -1, -1, -1],
#                 [-1, -1, -1, -1, 100, 100, -1, -1, -1]])
# at = a.T
# filter1 = operator(a.copy())
# filter2 = np.array(operator(at.copy())).T
# filter1[filter2 == 100] = filter2[filter2 == 100]
# print(a)
# print()
# print(filter1)
