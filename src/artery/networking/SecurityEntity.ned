package artery.networking;

simple SecurityEntity like ISecurityEntity
{
    parameters:
        @class(SecurityEntity);
        string runtimeModule;
        string positionModule;

        string CryptoBackend = default("Null"); //default Null
        string CertificateProvider = default("Naive"); //default Null OAM Naive
        string CertificateValidator = default("NullOk");
        string SignService = default("straight"); //default dummy OAM straight
        string VerifyService = default("straight"); //default dummy OAM straight
}
