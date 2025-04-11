# S3 File Copy Lambda with Metadata

This AWS Lambda function automatically copies files uploaded to one S3 bucket (the source) to another S3 bucket (the destination), adding specific metadata during the copy process.

## Purpose

When a file is uploaded to the source S3 bucket, this function is triggered. It performs the following actions:

1.  **Copies the file:** It copies the newly uploaded file to a designated destination S3 bucket.
2.  **Adds Metadata:** It adds or replaces metadata on the copied file in the destination bucket:
    *   `x-amz-meta-source-bucket`: Stores the name of the original bucket the file came from.
    *   `x-amz-meta-foxglove-device-name`: Extracts the first part of the file's path (before the first `/`) and uses it as the value for this metadata field. For example, if the file is `rby2/data/file.mcap`, the value will be `rby2`.

(Note: AWS S3 automatically adds the `x-amz-meta-` prefix to user-defined metadata keys.)

## Setup and Deployment

Follow these steps to deploy this function using the AWS Management Console:

1.  **Create Lambda Function:**
    *   Go to the AWS Lambda service in the console.
    *   Click "Create function".
    *   Choose "Author from scratch".
    *   Give your function a name (e.g., `s3-copy-with-metadata`).
    *   Select a **Runtime** (e.g., Python 3.9 or later).
    *   Choose or create an **Execution role**. This role needs specific permissions (see Permissions section below).
    *   Click "Create function".

2.  **Upload Code:**
    *   You need to package the `copy-s3-file.py` script along with its dependency (`boto3`).
    *   **Using Pipenv (Recommended):**
        *   Make sure you have `pipenv` installed (`pip install pipenv`).
        *   In the project directory (where `copy-s3-file.py` and `Pipfile` are), run:
            ```bash
            pipenv install
            pipenv lock -r > requirements.txt
            pip install -r requirements.txt -t ./package
            cp copy-s3-file.py ./package/
            cd package
            zip -r ../deployment-package.zip .
            cd ..
            ```
        *   This creates a `deployment-package.zip` file.
    *   **Alternatively (Manual):** Create a folder, put `copy-s3-file.py` inside it. Install `boto3` into the *same* folder: `pip install boto3 -t .`. Then zip the *contents* of the folder.
    *   In the Lambda function console, under the "Code" tab, click "Upload from" and select ".zip file". Upload the `deployment-package.zip` (or your manually created zip).
    *   Change the **Handler** setting under "Runtime settings" to `copy-s3-file.lambda_handler`.

3.  **Configure Environment Variables:**
    *   Go to the "Configuration" tab and select "Environment variables".
    *   Click "Edit".
    *   Add a new environment variable:
        *   **Key:** `DESTINATION_BUCKET_NAME`
        *   **Value:** Enter the exact name (not the ARN) of the S3 bucket where you want the files copied.
    *   Click "Save".

4.  **Set Permissions (IAM Role):**
    *   Go to the "Configuration" tab and select "Permissions".
    *   Click on the **Role name**. This will take you to the IAM console.
    *   Ensure the role has a policy attached that grants the following permissions:
        *   **Read from Source Bucket:** `s3:GetObject` permission for the source bucket (`arn:aws:s3:::<YOUR_SOURCE_BUCKET_NAME>/*`).
        *   **Write to Destination Bucket:** `s3:PutObject` and potentially `s3:PutObjectAcl` permissions for the destination bucket (`arn:aws:s3:::<YOUR_DESTINATION_BUCKET_NAME>/*`).
        *   **Logging (Optional but Recommended):** Permissions like `logs:CreateLogGroup`, `logs:CreateLogStream`, `logs:PutLogEvents` to allow the Lambda function to write logs to CloudWatch.
    *   You can add these permissions by editing an existing policy or creating a new inline policy for the role.

5.  **Configure Trigger (S3 Event Notification):**
    *   Go to the S3 service in the AWS console.
    *   Navigate to your **source** bucket.
    *   Go to the "Properties" tab.
    *   Scroll down to "Event notifications" and click "Create event notification".
    *   Give the notification a name (e.g., `trigger-lambda-copy`).
    *   Under "Event types", select "All object create events" or be more specific if needed (e.g., "PUT", "POST", "COPY").
    *   Under "Destination", choose "Lambda function" and select the function you created (`s3-copy-with-metadata`).
    *   Click "Save changes". AWS will automatically add the necessary permissions for S3 to invoke your Lambda function.

## How it Works

1.  A file is uploaded to the source S3 bucket.
2.  The S3 event notification triggers the Lambda function, passing information about the event (including the bucket name and file key) to the `event` parameter.
3.  The Python script (`copy-s3-file.py`) extracts the source bucket and key.
4.  It reads the `DESTINATION_BUCKET_NAME` from the environment variables.
5.  It constructs the metadata dictionary, including the source bucket and the extracted device name.
6.  It calls the S3 `copy_object` API using `boto3`, specifying the source, destination, metadata, and `MetadataDirective='REPLACE'` (to ensure the new metadata overwrites any existing metadata on the destination object if it happened to exist).
7.  The function logs success or failure messages to AWS CloudWatch Logs.

## Dependencies

*   **boto3:** The AWS SDK for Python, used to interact with S3.

This project uses `pipenv` for managing dependencies. A `Pipfile` and `Pipfile.lock` are included. 