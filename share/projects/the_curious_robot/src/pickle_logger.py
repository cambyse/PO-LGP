import datetime
import os
import pickle


class pickle_member(object):
    """
    A simple decorator that pickles the given instance variable
    `member_name` after calling the decorated function.

    Just decorate a function and supply the member_name.  Folders are
    automatically created.
    """
    def __init__(self, member_name):
        print member_name
        self.member_name = member_name

        # Create a folder for the logs
        now = datetime.datetime.now()
        self.foldername = "log/{}_{}_{}_{}_{}/{}".format(
            now.year, now.month, now.day, now.hour, now.minute,
            self.member_name,
        )
        try:
            os.makedirs(self.foldername)
        except OSError:
            print self.foldername, "already exists."

    def __call__(self, func):
        def wrapped(*args, **kwargs):
            result = func(*args, **kwargs)
            # pickle the data
            self._pickle(decorated_class=args[0])
            return result
        return wrapped

    def _pickle(self, decorated_class):
        """The pickling logic"""
        # we need the filename for the pkl file
        now = datetime.datetime.now()
        filename = "{}_{}_{}_{}_{}_{}_{}.pkl".format(
            now.year, now.month, now.day, now.hour, now.minute,
            now.second, now.microsecond
        )
        filename = os.path.join(self.foldername, filename)

        # pickle the instance variable
        member = getattr(decorated_class, self.member_name)
        with open(filename, "w") as f:
            pickle.dump(member, f)
