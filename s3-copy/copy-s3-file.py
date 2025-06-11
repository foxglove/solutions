import boto3
import json
import os

s3 = boto3.client('s3')

def lambda_handler(event, context):
    # Retrieve the source bucket and object key from the event
    source_bucket = event['Records'][0]['s3']['bucket']['name']
    source_key = event['Records'][0]['s3']['object']['key']

    # Define the destination bucket and key
    destination_bucket = os.environ.get('DESTINATION_BUCKET_NAME')
    if not destination_bucket:
        print("Error: DESTINATION_BUCKET_NAME environment variable not set.")
        return {
            'statusCode': 500,
            'body': json.dumps("Configuration error: Destination bucket not set.")
        }
    destination_key = source_key

    # Define the metadata to be set or modified
    metadata = {
        "source-bucket": source_bucket,
    }

    # Extract device name from the source key
    try:
        # Split the key and take the first part
        parts = source_key.split('/')
        if parts and parts[0]: # Check if split was successful and first part is not empty
            device_name = parts[0]
            # S3 metadata keys should be lowercase, although boto3 might handle casing.
            # Using lowercase consistently is safer.
            metadata["foxglove_device_name"] = device_name
        else:
            # Handle cases where the key might not have '/' or starts with '/'
            print(f"Warning: Could not extract device name from key: {source_key}")
    except Exception as e:
        # Catch potential errors during split or access
        print(f"Warning: Error extracting device name from key '{source_key}': {e}")

    # Copy the object with the specified metadata
    try:
        s3.copy_object(
            CopySource={'Bucket': source_bucket, 'Key': source_key},
            Bucket=destination_bucket,
            Key=destination_key,
            Metadata=metadata,
            MetadataDirective='REPLACE' # Use REPLACE to set exactly this metadata
        )
        return {
            'statusCode': 200,
            'body': json.dumps(f"Object copied successfully from {source_bucket}/{source_key} to {destination_bucket}/{destination_key}")
        }
    except Exception as e:
        print(e)
        return {
            'statusCode': 500,
            'body': json.dumps(f"Error copying object: {str(e)}")
        }