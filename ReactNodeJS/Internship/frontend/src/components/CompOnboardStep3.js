'use strict';

import React, { Component } from 'react';
import { withRouter, Link } from 'react-router-dom';
import Page from "./Page";

class CompOnboardStep3 extends Component {
    constructor(props) {
        super(props);

        this.state = {};
    }
    render() {
        // explicit class assigning based on validation

        return (
            <div className="step step4">
                <div className="row text-left" style={{paddingLeft:"50em", paddingRight:"50em"}}>
                    <form className="form-row">
                        <div className="col-md-12">
                            <p style={{fontSize :"1em"}}>Where is your company located? </p>
                        </div>
                        <div className="text-left col-md-6">
                            <input style={{width:"100%"}} placeholder="Street Name" />
                        </div>
                        <div className="text-left col-md-6" >
                            <input style={{width:"100%"}} placeholder="c/o" />
                        </div>
                        <div className="text-left col-md-4" style={{marginTop:"5em"}}>
                            <input style={{width:"100%"}} placeholder="Post Code" />
                        </div>
                        <div className="text-left col-md-4" style={{marginTop:"5em"}} >
                            <input style={{width:"100%"}} placeholder="City" />
                        </div>
                        <div className="text-left col-md-4" style={{marginTop:"5em"}} >
                            <input style={{width:"100%"}} placeholder="Country" />
                        </div>
                        <div className="col-md-12" style={{marginTop:"5em"}}>
                            <p style={{fontSize :"1em"}}>Tell us more about your company: </p>
                            <textarea className="text-left" style={{width: "100%", fontSIze: "0.8em", color: "grey"}}> </textarea>
                        </div>
                        <div className="text-left col-md-6" style={{marginTop:"5em"}} >
                            <input style={{width:"100%"}} placeholder="Industry" />
                        </div>
                        <div className="text-left col-md-6" style={{marginTop:"5em"}}>
                            <input style={{width:"100%"}} placeholder="Number of Employees" />
                        </div>
                        <div className="col-md-12" style={{marginTop:"5em"}}>
                            <input style={{width:"100%"}} placeholder="Website" />
                        </div>
                    </form>
                    </div>
            </div>
        )
    }
}
export default withRouter(CompOnboardStep3);
