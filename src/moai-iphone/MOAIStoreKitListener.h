//----------------------------------------------------------------//
// Copyright (c) 2010-2011 Zipline Games, Inc. 
// All Rights Reserved. 
// http://getmoai.com
//----------------------------------------------------------------//

#ifndef DISABLE_BILLING

#import <Foundation/Foundation.h>
#import <StoreKit/StoreKit.h>
#import <UIKit/UIKit.h>

//================================================================//
// MOAIStoreKitListener
//================================================================//
@interface MOAIStoreKitListener : NSObject < SKProductsRequestDelegate, SKPaymentTransactionObserver > {
@private
}

@end

#endif // DISABLE_BILLING