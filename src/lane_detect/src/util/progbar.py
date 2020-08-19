import sys
import time
import numpy as np
import torch

class progbar(object):
    """Displays a progress bar.
    # Arguments
        target: Total number of steps expected, None if unknown.
        interval: Minimum visual progress update interval (in seconds).
    """

    def __init__(self, target, width=30, verbose=1, interval=0.05):
        self.width = width
        if target is None:
            target = -1
        self.target = target
        self.sum_values = {}
        self.unique_values = []
        self.start = time.time()
        self.last_update = 0
        self.interval = interval
        self.total_width = 0
        self.seen_so_far = 0
        self.verbose = verbose

    def update(self, current, values=None, force=False):
        """Updates the progress bar.
        # Arguments
            current: Index of current step.
            values: List of tuples (name, value_for_last_step).
                The progress bar will display averages for these values.
            force: Whether to force visual progress update.
        """
        values = values or []
        for k, v in values:
            if k not in self.sum_values:
                self.sum_values[k] = [v * (current - self.seen_so_far),
                                      current - self.seen_so_far]
                self.unique_values.append(k)
            else:
                #if type(self.sum_values[k][0]) != type(1.0):
                if torch.is_tensor(self.sum_values[k][0]):
                    self.sum_values[k][0] = self.sum_values[k][0].cpu().numpy()

                x = v * (current - self.seen_so_far)

                if torch.is_tensor(x):
                    x = x.cpu().numpy()

                #self.sum_values[k][0] = self.sum_values[k][0].cpu().numpy()
#                 print("v", type(v * (current - self.seen_so_far)))
#                 print("self.seen_so_far : ", type(self.seen_so_far))

                self.sum_values[k][0] += x
                self.sum_values[k][1] += (current - self.seen_so_far)
        self.seen_so_far = current

        now = time.time()
        if self.verbose == 1:
            if not force and (now - self.last_update) < self.interval:
                return

            prev_total_width = self.total_width
            sys.stdout.write('\b' * prev_total_width)
            sys.stdout.write('\r')

            if self.target is not -1:
                numdigits = int(np.floor(np.log10(self.target))) + 1
                barstr = '%%%dd/%%%dd [' % (numdigits, numdigits)
                bar = barstr % (current, self.target)
                prog = float(current) / self.target
                prog_width = int(self.width * prog)
                if prog_width > 0:
                    bar += ('=' * (prog_width - 1))
                    if current < self.target:
                        bar += '>'
                    else:
                        bar += '='
                bar += ('.' * (self.width - prog_width))
                bar += ']'
                sys.stdout.write(bar)
                self.total_width = len(bar)

            if current:
                time_per_unit = (now - self.start) / current
            else:
                time_per_unit = 0
            eta = time_per_unit * (self.target - current)
            info = ''
            if current < self.target and self.target is not -1:
                info += '  ETA: %ds' % eta
            else:
                info += '  %ds' % (now - self.start)
            for k in self.unique_values:
                info += '  %s:' % k
                if isinstance(self.sum_values[k], list):
                    x_1 = self.sum_values[k][0]
                    if torch.is_tensor(x_1):
                        self.sum_values[k][0] = self.sum_values[k][0].cpu().numpy()
                    # print("self.sum_values[k][0] : ",type(self.sum_values[k][0]))  ## float
                    # print("self.sum_values[k][1] : ",type(self.sum_values[k][1]))  ## int
                    #if (type(self.sum_values[k][1]) != type(1.0)) or (type(self.sum_values[k][1]) != type(1)):
                        #self.sum_values[k][1] = self.sum_values[k][1].cpu().numpy()

                    avg = np.mean(np.array(self.sum_values[k][0] / max(1, self.sum_values[k][1])))
                    # avg = (self.sum_values[k][0] / max(1, self.sum_values[k][1])).mean(axis=0)
                    #avg = 1
                    if abs(avg) > 1e-3:
                        info += ' %.4f' % avg
                    else:
                        info += ' %.4e' % avg
                else:
                    info += ' %s' % self.sum_values[k]

            self.total_width += len(info)
            if prev_total_width > self.total_width:
                info += ((prev_total_width - self.total_width) * ' ')

            sys.stdout.write(info)
            sys.stdout.flush()

            if current >= self.target:
                sys.stdout.write('\n')

        if self.verbose == 2:
            if current >= self.target:
                info = '%ds' % (now - self.start)
                for k in self.unique_values:
                    info += ' - %s:' % k
                    avg = np.mean(self.sum_values[k][0] / max(1, self.sum_values[k][1]))
                    if avg > 1e-3:
                        info += ' %.4f' % avg
                    else:
                        info += ' %.4e' % avg
                sys.stdout.write(info + "\n")

        self.last_update = now

    def add(self, n, values=None):
        self.update(self.seen_so_far + n, values)
