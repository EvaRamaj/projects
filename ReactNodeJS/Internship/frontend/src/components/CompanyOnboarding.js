"use strict";

import React from 'react';
import { withRouter } from 'react-router-dom';
import StepZilla from "react-stepzilla";
import CompOnboardStep1 from './CompOnboardStep1';
import CompOnboardStep2 from './CompOnboardStep2';
import CompOnboardStep3 from './CompOnboardStep3';
import Page from './Page';

class CompanyOnboarding extends React.Component
{
    constructor(props) {
        super(props);
    }

    render() {
        const steps =
            [
                {name: 'Project', component: <CompOnboardStep1 />},
                {name: 'Skills', component: <CompOnboardStep2 />},
                {name: 'Company', component: <CompOnboardStep3 />}

            ];

        return (
            <Page>
                <h1 className="text-center">My Profile</h1>
                <h4 className="text-center">You are 1 step away to be matched with interesting projects</h4>
                <div className='step-progress'>
                    <StepZilla steps={steps} nextTextOnFinalActionStep={"Save"}/>
                </div>
            </Page>
        );
    }
};

export default withRouter(CompanyOnboarding);
