from qdrant_client import QdrantClient

qdrant_client = QdrantClient(
    url="https://932c4c57-6c35-4662-baeb-5091eb108df1.europe-west3-0.gcp.cloud.qdrant.io:6333", 
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.0TlxXuRoZPYH_ClWklI_BDoQRK2-tTk-EapX5i--Ei4",
)

print(qdrant_client.get_collections())