# Minimum Permissions for Agent 

## Create user
aws iam create-user --user-name s3-writer

## Attach inline policy
aws iam put-user-policy \
  --user-name s3-writer \
  --policy-name s3-write-only \
  --policy-document '{
    "Version": "2012-10-17",
    "Statement": [
      {
        "Effect": "Allow",
        "Action": "s3:PutObject",
        "Resource": "arn:aws:s3:::fg-demo-inbox-bucket/*"
      }
    ]
  }'

## Create access keys
aws iam create-access-key --user-name s3-writer


## Run Agent in Docker 

docker volume create foxglove-agent-index-storage
docker run \
  --mount type=volume,src=foxglove-agent-index-storage,dst=/index \
  --mount type=bind,src=./recordings,dst=/storage \
  --env-file agent_config.env \
    us-central1-docker.pkg.dev/foxglove-images/images/agent