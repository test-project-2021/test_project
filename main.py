from tests.validator import Validator
print("hi")
username = 'Roman'
validator = Validator()
print("hi")
if validator.username_is_valid(username):
    print("username is valid")
else:
    print("username is not valid")