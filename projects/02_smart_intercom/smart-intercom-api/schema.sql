create table push_subscription (
	id int primary key,
	application_id int,
	subscription_data json
);