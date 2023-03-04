import numpy as np
from typing import Union

def project_vec_to_plane(v: np.ndarray, n: np.ndarray, normalize = True) -> np.ndarray:
    result = v - np.dot(v, n) / (np.linalg.norm(n) ** 2) * n
    return result / np.linalg.norm(result) if normalize else result

def intersect_vec_plane(p: np.ndarray, n: np.ndarray, pv: np.ndarray, v: np.ndarray,) -> Union[np.ndarray, None]:
    a = np.dot(n, v)
    if a != 0:
        return pv + np.dot((p - pv), n) * v / a
    else:
        None

def normalize_vec(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)

class BinaryFilter(object):
    '''
    Filtering binary state streams by converting them to continuous values 
    '''
    def __init__(self, init_val: bool, filter_co: float = 0.25, filter_T_co: float = -1, filter_F_co = -1) -> None:
        #self._F2T_threshold = 0.7
        #self._T2F_threshold = 0.3
        self._val = init_val ## Might be useful if we want to use two different thresholds for state transitions
        self._cont_val = 1.0 if init_val else 0.0
        self._biased = filter_T_co > 0 and filter_F_co > 0
        if self._biased:
            self._filter_T_co = filter_T_co # how much we trust new true value
            self._filter_F_co = filter_F_co # how much we trust new false value
        else:
            self._filter_co = filter_co # how much we trust new value

    def update(self, new_val: bool) -> bool:
        filter_co = (self._filter_T_co if new_val else self._filter_F_co) if self._biased else self._filter_co
        self._cont_val = self._cont_val * (1 - filter_co) + (1.0 if new_val else 0.0) * filter_co
        return self._cont_val >= 0.5

    def get_val(self) -> bool:
        return self._cont_val >= 0.5

    def hard_update(self, new_val: bool) -> bool:
        self._cont_val = 1.0 if new_val else 0.0
        return self._cont_val >= 0.5

class Vector3Filter(object):
    def __init__(self, init_val: Union[np.ndarray, None], filter_co = 0.25, normalize = True) -> None:
        self._val = init_val
        self._co = filter_co
        self._normalize = normalize

    def update(self, new_val: np.ndarray) -> np.ndarray:
        if self._val is None:
            self._val = new_val
        else:
            after_update = self._val * (1 - self._co) + new_val * self._co
            after_update = normalize_vec(after_update) if self._normalize else after_update
            self._val = after_update
        return self._val

    def get_val(self) -> Union[np.ndarray, None]:
        return self._val

    def hard_update(self, new_val: np.ndarray) -> np.ndarray:
        self._val = new_val
        return self._val