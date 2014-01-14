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
        self.member_name = member_name

        # Create a folder for the logs
        self.foldername = datetime.datetime.now().strftime(
            "log_tcr/%Y_%m_%d_%H_%M/" + self.member_name
        )
        print "Initializing pickler for {}; will be saved in {}.".format(
            member_name, self.foldername)
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
        # construct filename for pkl file
        filename = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%f.pkl")
        filename = os.path.join(self.foldername, filename)

        # pickle the instance variable
        member = getattr(decorated_class, self.member_name)
        with open(filename, "w") as f:
            pickle.dump(member, f)
