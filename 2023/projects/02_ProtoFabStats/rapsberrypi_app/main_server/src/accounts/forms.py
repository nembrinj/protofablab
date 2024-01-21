from flask_wtf import FlaskForm
from wtforms import EmailField, PasswordField
from wtforms.validators import DataRequired, Email, EqualTo, Length, ValidationError
from src.models.user import User


class _EmailDomainValidator:
    def __init__(self, allowlisted_domains, message=None):
        self.allowlisted_domains = allowlisted_domains
        self.message = message
        self.field_flags = {"required": True}


    def __call__(self, _, field):
        print(field.data)
        domain = field.data.split('@')[-1]
        print(domain)
        if domain not in self.allowlisted_domains:
            message = "Email does not have an approved domain. Approved domains are " +\
                ", ".join([f"\"{domain}\"" for domain in self.allowlisted_domains]) +\
                "."
            raise ValidationError(message)



class LoginForm(FlaskForm):
    email = EmailField("Email", validators=[DataRequired(), Email()])
    password = PasswordField("Password", validators=[DataRequired()])


class RegisterForm(FlaskForm):
    ALLOWLISTED_DOMAINS = ["students.unibe.ch", "unifr.ch", "unine.ch"]
    FORMATTED_ALLOWLISTED_DOMAINS = ", ".join([f"\"{domain}\"" for domain in ALLOWLISTED_DOMAINS])

    email = EmailField(
        "Email", validators=[DataRequired(), Email(message=None), Length(min=6, max=40), _EmailDomainValidator(ALLOWLISTED_DOMAINS)]
    )
    password = PasswordField(
        "Password", validators=[DataRequired(), Length(min=6, max=25)]
    )
    confirm = PasswordField(
        "Repeat password",
        validators=[
            DataRequired(),
            EqualTo("password", message="Passwords must match."),
        ],
    )

    def validate(self, _):
        initial_validation = super(RegisterForm, self).validate()
        if not initial_validation:
            return False
        user = User.query.filter_by(email=self.email.data).first()
        if user:
            self.email.errors.append("Email already registered")
            return False
        if self.password.data != self.confirm.data:
            self.password.errors.append("Passwords must match")
            return False
        return True
