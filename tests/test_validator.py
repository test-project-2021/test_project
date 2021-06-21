import unittest
from validator import Validator

class test_validator(unittest.TestCase):
    def test_reject_user_name_if_long(self):
        #Assume
        username= 'InvalidTooLong'
        validator = Validator()

        #Action
        result = validator.username_is_valid(username)

        #Assert
        self.assertFalse(result)

    def  test_reject_user_name_if_space(self):
        #Assume
        username= 'roman'
        validator = Validator()


        #Action
        result = validator.username_is_valid(username)

        #Assert
        self.assertFalse(result)